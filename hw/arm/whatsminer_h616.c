/*
 * Whatsminer / Allwinner H616 (sun50iw9) control board
 *
 * A QEMU machine for the MicroBT Whatsminer control-board vendor kernel
 * (Allwinner H616, Cortex-A53, Linux 4.9.170). It models just enough of the
 * SoC for that kernel to boot to userspace with a live serial console:
 *
 *   - Cortex-A53 CPU(s) with emulated PSCI,
 *   - a GIC-400 (GICv2) at the H616 addresses,
 *   - the ARM architected timer,
 *   - the Allwinner "sun50i-uart" (a DesignWare 8250 == 16550) at 0x05000000,
 *   - 256MiB+ DRAM at 0x40000000,
 *   - benign backing for the CCU register windows so clock probing does not
 *     fault or spin.
 *
 * The board is driven by the *real* vendor device tree (pass it with -dtb;
 * grab it from a running miner via `cp /sys/firmware/fdt board.dtb`), so all
 * addresses/IRQs/UART bindings come from the hardware itself. arm_load_kernel()
 * fills in the /memory, /chosen (bootargs + initrd) and /psci nodes.
 *
 * The one thing a stock 16550 lacks that this kernel needs is the DesignWare
 * UART Status Register (USR, offset 0x7C): earlyprintk=sunxi-uart and the
 * vendor console poll its TFNF/TFE bits before writing THR. We place the 16550
 * plus a small USR shim (synthesised from the 16550 LSR) at 0x05000000.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/core/boards.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/arm/boot.h"
#include "hw/arm/bsa.h"
#include "hw/intc/arm_gic.h"
#include "hw/intc/arm_gic_common.h"
#include "hw/char/serial-mm.h"
#include "hw/sd/allwinner-sdhost.h"
#include "hw/sd/sd.h"
#include "hw/net/allwinner-sun8i-emac.h"
#include "net/net.h"
#include "system/system.h"
#include "system/blockdev.h"
#include "system/block-backend.h"
#include "system/address-spaces.h"
#include "target/arm/cpu.h"
#include "target/arm/gtimer.h"
#include "qom/object.h"
#include <libfdt.h>

/* --- H616 memory map (from the vendor device tree) --- */
#define H616_DRAM_BASE      0x40000000
#define H616_GIC_DIST_BASE  0x03021000
#define H616_GIC_DIST_SIZE  0x1000
#define H616_GIC_CPU_BASE   0x03022000
#define H616_GIC_CPU_SIZE   0x2000
#define H616_UART0_BASE     0x05000000
#define H616_UART0_IRQ      0          /* GIC SPI 0 (DT: interrupts = <0 0 4>) */
#define H616_UART_CLK       24000000   /* 24 MHz APB */
#define H616_UART_REGSHIFT  2

/* GIC line budget: 32 internal + this many SPIs (H616 uses < 224). */
#define H616_NUM_IRQS       256

#define H616_MAX_CPUS       4

/*
 * CCU / PRCM register windows the vendor clock driver ("allwinner,clk-init")
 * touches during probe. We don't model the clock tree; we just make these reads
 * return all-ones so any "PLL locked / clock ready" poll succeeds instead of
 * spinning, and swallow writes. (reg from the DT /clocks node.)
 */
static const struct { hwaddr base, size; } h616_ccu_windows[] = {
    { 0x03001000, 0x1000 },   /* CCU        */
    { 0x07000000, 0x1000 },   /* R_PRCM / R_CPUCFG area */
    { 0x07010000, 0x1000 },   /* R_PRCM     */
};

static uint64_t h616_ccu_read(void *opaque, hwaddr off, unsigned size)
{
    if (getenv("H616_CCU_TRACE")) {
        fprintf(stderr, "[ccu] RD  base=%#lx off=%#lx -> ~0\n",
                (unsigned long)(uintptr_t)opaque, (unsigned long)off);
    }
    return ~(uint64_t)0;      /* every status/lock bit reads as ready */
}
static void h616_ccu_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    if (getenv("H616_CCU_TRACE")) {
        fprintf(stderr, "[ccu] WR  base=%#lx off=%#lx <- %#lx\n",
                (unsigned long)(uintptr_t)opaque, (unsigned long)off,
                (unsigned long)val);
    }
    /* clock/PLL programming: accept and ignore */
}
static const MemoryRegionOps h616_ccu_ops = {
    .read = h616_ccu_read,
    .write = h616_ccu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.max_access_size = 4,
    .impl.max_access_size = 4,
};

/*
 * --- Allwinner H616 PIO (pin controller) as a plain RW register bank ---
 * We don't route pins anywhere: the SD card is wired straight to the SD-host
 * model and the console straight to the UART. But the vendor sunxi pinctrl
 * driver, when the driver core applies a device's "default" pin state
 * (pinctrl_bind_pins, run *before* ->probe), writes the pin's mux/function bits
 * and then reads them back to verify. With no PIO model those reads hit dead
 * MMIO, the mux read-back check fails, and e.g. sunxi-mmc's probe never runs
 * ("probe of sdc0 failed with error -1", no driver banner). A register file
 * that simply returns what was written makes pinctrl_bind_pins succeed. This is
 * register-bank behaviour, not clock/timing emulation.
 */
typedef struct { uint32_t regs[0x1000 / 4]; } H616Pio;

static uint64_t h616_pio_read(void *opaque, hwaddr off, unsigned size)
{
    H616Pio *s = opaque;
    return s->regs[(off & 0xfff) >> 2];
}
static void h616_pio_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    H616Pio *s = opaque;
    s->regs[(off & 0xfff) >> 2] = (uint32_t)val;
}
static const MemoryRegionOps h616_pio_ops = {
    .read = h616_pio_read,
    .write = h616_pio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.max_access_size = 4,
    .impl.max_access_size = 4,
};

static void h616_create_pio(MemoryRegion *sysmem, hwaddr base)
{
    H616Pio *s = g_new0(H616Pio, 1);
    MemoryRegion *mr = g_new(MemoryRegion, 1);
    char *nm = g_strdup_printf("h616-pio@%" HWADDR_PRIx, base);

    memory_region_init_io(mr, NULL, &h616_pio_ops, s, nm, 0x1000);
    memory_region_add_subregion(sysmem, base, mr);
    g_free(nm);
}

/* --- Allwinner sun50i-uart: 16550 + DesignWare USR@0x7C --- */
/* DesignWare USR bits. */
#define USR_BUSY 0x01
#define USR_TFNF 0x02
#define USR_TFE  0x04
#define USR_RFNE 0x08
/* 16550 LSR bits. */
#define LSR_DR   0x01
#define LSR_THRE 0x20
#define LSR_TEMT 0x40

static uint64_t h616_uart_ext_read(void *opaque, hwaddr off, unsigned size)
{
    SerialMM *smm = opaque;
    hwaddr abs = off + 0x20;              /* this MR starts at container +0x20 */

    if (abs == 0x7c) {                    /* USR, derived from the 16550 LSR */
        uint8_t lsr = smm->serial.lsr;
        uint64_t v = 0;
        if (lsr & LSR_THRE) v |= USR_TFNF;
        if (lsr & LSR_TEMT) v |= USR_TFE;
        if (lsr & LSR_DR)   v |= USR_RFNE;
        return v;
    }
    return 0;
}
static void h616_uart_ext_write(void *opaque, hwaddr off, uint64_t val,
                                unsigned size)
{
    /* DesignWare-only registers (SRR reset, HALT, DLF, ...): no-op. */
}
static const MemoryRegionOps h616_uart_ext_ops = {
    .read = h616_uart_ext_read,
    .write = h616_uart_ext_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.max_access_size = 4,
    .impl.max_access_size = 4,
};

static void h616_create_uart(DeviceState *gic, MemoryRegion *sysmem)
{
    hwaddr base = H616_UART0_BASE;
    DeviceState *dev = qdev_new(TYPE_SERIAL_MM);
    SysBusDevice *s = SYS_BUS_DEVICE(dev);
    SerialMM *smm = SERIAL_MM(dev);
    MemoryRegion *container = g_new(MemoryRegion, 1);
    MemoryRegion *ext = g_new(MemoryRegion, 1);

    qdev_prop_set_uint8(dev, "regshift", H616_UART_REGSHIFT);
    qdev_prop_set_uint32(dev, "baudbase", H616_UART_CLK);
    qdev_prop_set_chr(dev, "chardev", serial_hd(0));
    qdev_prop_set_uint8(dev, "endianness", DEVICE_LITTLE_ENDIAN);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_connect_irq(s, 0, qdev_get_gpio_in(gic, H616_UART0_IRQ));

    /* 0x400 window: 16550 block at 0x00, DesignWare USR shim above it. */
    memory_region_init(container, OBJECT(dev), "sun50i-uart", 0x400);
    memory_region_add_subregion(container, 0, sysbus_mmio_get_region(s, 0));
    memory_region_init_io(ext, OBJECT(dev), &h616_uart_ext_ops, smm,
                          "sun50i-uart-dw", 0x400 - 0x20);
    memory_region_add_subregion(container, 0x20, ext);
    memory_region_add_subregion(sysmem, base, container);
}

/* --- GIC-400 (GICv2), wired like hw/arm/virt.c's v2 path --- */
/*
 * SD/MMC controller at the H616 SDMMC0 address (0x04020000, IRQ SPI 35, the
 * "status okay" v4p1x slot in the vendor DTB). Modelled with QEMU's Allwinner
 * SD-host (sun50i-a64 is the closest generation). An SD card backed by the
 * assembled NAND image (-sd/-drive if=sd) shows up as /dev/mmcblk0pX, so the
 * initramfs can dm-crypt mmcblk0p4 and mount the real partitions.
 */
#define H616_MMC0_BASE 0x04020000
#define H616_MMC0_IRQ  35            /* GIC SPI 35 (DT: interrupts = <0 0x23 4>) */

static void h616_create_mmc(MachineState *machine, DeviceState *gic,
                            MemoryRegion *sysmem)
{
    DeviceState *mmc = qdev_new(TYPE_AW_SDHOST_SUN50I_H616);
    DriveInfo *di;

    object_property_add_child(OBJECT(machine), "mmc0", OBJECT(mmc));
    object_property_set_link(OBJECT(mmc), "dma-memory", OBJECT(sysmem),
                             &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(mmc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(mmc), 0, H616_MMC0_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(mmc), 0,
                       qdev_get_gpio_in(gic, H616_MMC0_IRQ));

    di = drive_get(IF_SD, 0, 0);
    if (di) {
        BusState *bus = qdev_get_child_bus(mmc, "sd-bus");
        DeviceState *card = qdev_new(TYPE_SD_CARD);
        qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di),
                                &error_fatal);
        qdev_realize_and_unref(card, bus, &error_fatal);
    }
}

/*
 * Ethernet: the H616 "gmac1" (eth@0x05030000, IRQ SPI 15, RMII, status=okay in
 * the vendor DTB) is Allwinner's sun8i EMAC -- the same IP QEMU models as
 * TYPE_AW_SUN8I_EMAC. The board is headless (no serial getty); networking is
 * how the vendor userspace (dropbear SSH, the uhttpd/luci web UI) is reached, so
 * we model it and pair it with a -nic. gmac0 (0x05020000) stays disabled (as in
 * the DTB). The EMAC's PHY-select/delay live in a syscon register at
 * 0x03000030+; QEMU's EMAC model doesn't use it, so a plain RW bank suffices
 * (see h616_create_pio, reused for 0x03000000).
 */
#define H616_EMAC1_BASE 0x05030000
#define H616_EMAC1_IRQ  15           /* GIC SPI 15 (DT: interrupts = <0 15 4>) */

static void h616_create_emac(MachineState *machine, DeviceState *gic,
                             MemoryRegion *sysmem)
{
    DeviceState *emac = qdev_new(TYPE_AW_SUN8I_EMAC);

    object_property_add_child(OBJECT(machine), "emac1", OBJECT(emac));
    qemu_configure_nic_device(emac, true, NULL);
    object_property_set_link(OBJECT(emac), "dma-memory", OBJECT(sysmem),
                             &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(emac), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(emac), 0, H616_EMAC1_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(emac), 0,
                       qdev_get_gpio_in(gic, H616_EMAC1_IRQ));
}

static DeviceState *h616_create_gic(MachineState *ms, MemoryRegion *sysmem)
{
    unsigned int smp_cpus = ms->smp.cpus;
    DeviceState *gic = qdev_new(gic_class_name());
    SysBusDevice *gicbusdev = SYS_BUS_DEVICE(gic);
    int i;

    qdev_prop_set_uint32(gic, "revision", 2);
    qdev_prop_set_uint32(gic, "num-cpu", smp_cpus);
    /* num-irq counts 32 internal + external; must cover H616_NUM_IRQS SPIs. */
    qdev_prop_set_uint32(gic, "num-irq", H616_NUM_IRQS + 32);
    qdev_prop_set_bit(gic, "has-security-extensions", false);
    qdev_prop_set_bit(gic, "has-virtualization-extensions", false);
    sysbus_realize_and_unref(gicbusdev, &error_fatal);

    sysbus_mmio_map(gicbusdev, 0, H616_GIC_DIST_BASE);
    sysbus_mmio_map(gicbusdev, 1, H616_GIC_CPU_BASE);

    /* Wire each CPU's timer outputs to GIC PPIs and GIC IRQ/FIQ back to CPU. */
    for (i = 0; i < smp_cpus; i++) {
        DeviceState *cpudev = DEVICE(qemu_get_cpu(i));
        int intidbase = H616_NUM_IRQS + i * GIC_INTERNAL;
        const int timer_irq[] = {
            [GTIMER_PHYS] = ARCH_TIMER_NS_EL1_IRQ,
            [GTIMER_VIRT] = ARCH_TIMER_VIRT_IRQ,
            [GTIMER_HYP]  = ARCH_TIMER_NS_EL2_IRQ,
            [GTIMER_SEC]  = ARCH_TIMER_S_EL1_IRQ,
        };
        unsigned irq;

        for (irq = 0; irq < ARRAY_SIZE(timer_irq); irq++) {
            qdev_connect_gpio_out(cpudev, irq,
                                  qdev_get_gpio_in(gic,
                                                   intidbase + timer_irq[irq]));
        }

        sysbus_connect_irq(gicbusdev, i,
                           qdev_get_gpio_in(cpudev, ARM_CPU_IRQ));
        sysbus_connect_irq(gicbusdev, i + smp_cpus,
                           qdev_get_gpio_in(cpudev, ARM_CPU_FIQ));
        sysbus_connect_irq(gicbusdev, i + 2 * smp_cpus,
                           qdev_get_gpio_in(cpudev, ARM_CPU_VIRQ));
        sysbus_connect_irq(gicbusdev, i + 3 * smp_cpus,
                           qdev_get_gpio_in(cpudev, ARM_CPU_VFIQ));
    }
    return gic;
}

/*
 * The vendor DTB enumerates the whole SoC. We only model the CPU/GIC/timer and
 * the UART, so the drivers for the unmodeled buses (I2C PMU, NAND, MMC, USB,
 * SPI, GMAC, ...) probe against dead MMIO and burn many seconds in per-device
 * timeouts (or hang). None of them are needed to boot to the initramfs, so mark
 * their nodes disabled. Matching is by a keyword in the node's "compatible".
 */
static const char * const h616_disable_kw[] = {
    "-twi", "i2c",          /* sunxi TWI + i2c-gpio (AXP PMU lives here) */
    "-nand",                /* raw NAND */
    /* NB: MMC is left enabled — we model sdmmc@0x04020000 so mmcblk0 works;
     * the other sdmmc nodes are already status=disabled in the vendor DTB. */
    "ehci", "ohci", "otg", "usbstandby", /* USB */
    "-spi",                 /* SPI (won't match "hwspinlock") */
    "hwspinlock",
    /* NB: ethernet (gmac) is left enabled — we model gmac1 (sun8i EMAC) at
     * 0x05030000; gmac0 is already status=disabled in the vendor DTB. */
};

static void h616_modify_dtb(const struct arm_boot_info *info, void *fdt)
{
    int off;

    for (off = fdt_next_node(fdt, -1, NULL); off >= 0;
         off = fdt_next_node(fdt, off, NULL)) {
        int len;
        const char *compat = fdt_getprop(fdt, off, "compatible", &len);
        const char *p;
        int i;

        if (!compat) {
            continue;
        }
        /* "compatible" may be a list of NUL-separated strings. */
        for (p = compat; p < compat + len; p += strlen(p) + 1) {
            for (i = 0; i < ARRAY_SIZE(h616_disable_kw); i++) {
                if (strstr(p, h616_disable_kw[i])) {
                    fdt_setprop_string(fdt, off, "status", "disabled");
                    goto next_node;
                }
            }
            /*
             * The MMC controller (sunxi-mmc-v4p1x) keeps status=okay -- we model
             * it, and its pinctrl is left intact: the vendor driver dereferences
             * host->pinctrl unconditionally after devm_pinctrl_get(), so the node
             * MUST resolve to a real (if unused) pinctrl handle. The pin mux
             * itself is irrelevant in QEMU (the SD card is wired straight to the
             * SD-host model); the "expect_func" pin warnings are harmless.
             */
        }
    next_node: ;
    }
}

static struct arm_boot_info h616_binfo;

static void whatsminer_h616_init(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();
    DeviceState *gic;
    int n;

    if (machine->ram_size < 256 * MiB) {
        error_report("whatsminer-h616 needs at least 256MiB of RAM");
        exit(1);
    }
    if (!machine->dtb) {
        error_report("whatsminer-h616 requires the vendor device tree: "
                     "-dtb <board.dtb> (grab it from a running miner with "
                     "`cp /sys/firmware/fdt board.dtb`)");
        exit(1);
    }

    /* CPUs: Cortex-A53, no EL3, PSCI via HVC (emulated by QEMU). */
    for (n = 0; n < machine->smp.cpus; n++) {
        Object *cpu = object_new(machine->cpu_type);

        /*
         * No EL3 and no EL2 -> Linux boots at EL1 and QEMU's emulated PSCI
         * intercepts the PSCI SMC directly (arm_is_psci_call just matches the
         * SMC conduit; with EL3 present the SMC would instead trap to the
         * unmodelled EL3 firmware). QEMU answers PSCI function IDs and returns
         * NOT_SUPPORTED for the vendor's other SMCs, so nothing wedges. (This
         * needs a kernel whose generic SMC helper is NOT stubbed out; see
         * patch-kernel.py --keep-smc / kernel_image.h616.)
         */
        if (object_property_find(cpu, "has_el3")) {
            object_property_set_bool(cpu, "has_el3", false, &error_fatal);
        }
        if (object_property_find(cpu, "has_el2")) {
            object_property_set_bool(cpu, "has_el2", false, &error_fatal);
        }
        if (object_property_find(cpu, "reset-cbar")) {
            object_property_set_int(cpu, "reset-cbar", H616_GIC_DIST_BASE,
                                    &error_abort);
        }
        qdev_realize(DEVICE(cpu), NULL, &error_fatal);
        object_unref(cpu);
    }

    /* DRAM at 0x40000000. */
    memory_region_add_subregion(sysmem, H616_DRAM_BASE, machine->ram);

    /* Benign CCU/PRCM backing (clock probe must not fault or spin). */
    for (n = 0; n < ARRAY_SIZE(h616_ccu_windows); n++) {
        MemoryRegion *mr = g_new(MemoryRegion, 1);
        char *nm = g_strdup_printf("h616-ccu@%" HWADDR_PRIx,
                                   h616_ccu_windows[n].base);
        memory_region_init_io(mr, NULL, &h616_ccu_ops,
                              (void *)(uintptr_t)h616_ccu_windows[n].base, nm,
                              h616_ccu_windows[n].size);
        memory_region_add_subregion(sysmem, h616_ccu_windows[n].base, mr);
        g_free(nm);
    }

    /* PIO / R_PIO register banks (pinctrl mux read-back must be coherent). */
    h616_create_pio(sysmem, 0x0300b000);   /* CPUX PIO  (ports PC..PI) */
    h616_create_pio(sysmem, 0x07022000);   /* R_PIO     (ports PL..PM) */
    h616_create_pio(sysmem, 0x03000000);   /* system-control / EMAC clk reg */

    gic = h616_create_gic(machine, sysmem);
    h616_create_uart(gic, sysmem);
    h616_create_mmc(machine, gic, sysmem);
    h616_create_emac(machine, gic, sysmem);

    /* Boot the vendor kernel with the vendor DTB; QEMU emulates PSCI. */
    h616_binfo.ram_size = machine->ram_size;
    h616_binfo.loader_start = H616_DRAM_BASE;
    h616_binfo.board_id = -1;
    /*
     * SMC, not HVC: the A53 has EL2 (no EL3), so Linux boots at EL2/EL1 with
     * its own hyp stub installed. A PSCI call via HVC would hit that stub, not
     * QEMU's emulated PSCI ("Conflicting PSCI version / PSCIv0.0"). With no EL3,
     * QEMU intercepts the PSCI SMC at any EL, so SMC is the right conduit (this
     * is what the in-tree Allwinner boards use too).
     */
    h616_binfo.psci_conduit = QEMU_PSCI_CONDUIT_SMC;
    h616_binfo.modify_dtb = h616_modify_dtb;
    arm_load_kernel(ARM_CPU(first_cpu), machine, &h616_binfo);
}

static void whatsminer_h616_machine_init(MachineClass *mc)
{
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-a53"),
        NULL
    };

    mc->desc = "Whatsminer / Allwinner H616 control board (Cortex-A53)";
    mc->init = whatsminer_h616_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a53");
    mc->valid_cpu_types = valid_cpu_types;
    mc->max_cpus = H616_MAX_CPUS;
    mc->default_cpus = H616_MAX_CPUS;   /* real board is a quad-core A53 */
    mc->min_cpus = 1;
    mc->default_ram_size = 1 * GiB;
    mc->default_ram_id = "whatsminer-h616.ram";
    /* The full vendor DTB enumerates many blocks we don't model; let stray
     * MMIO accesses read-as-zero / write-ignore instead of faulting. */
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE("whatsminer-h616", whatsminer_h616_machine_init)
