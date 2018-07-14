#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/backlight.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/types.h>

#include <asm/page.h>

#define FPGA_REGS_BASE                    (0xff200000) // lwhps2fpga
#define REGSIZE                           (4)

static u32 seven_inch_lcd_fb_pseudo_palette[16];
void *fpga_regs;

static void fpga_write_reg(int reg, u32 val)
{
	iowrite32(val, fpga_regs + 4*reg);
	// *(uint32_t*)(fpga_regs + 4*reg) = val;
}

#define CNVT_TOHW(val, width) ((((val)<<(width))+0x7FFF-(val))>>16)

static int seven_inch_lcd_fb_setcolreg(unsigned regno,
			    unsigned red, unsigned green, unsigned blue,
			    unsigned transp, struct fb_info *info)
{
	int ret = 1;

	if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
				      7471 * blue) >> 16;

	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		if (regno < 16) {
			u32 *pal = info->pseudo_palette;
			u32 value;

			// red = CNVT_TOHW(red, info->var.red.length);
			// green = CNVT_TOHW(green, info->var.green.length);
			// blue = CNVT_TOHW(blue, info->var.blue.length);
			// transp = CNVT_TOHW(transp, info->var.transp.length);

			value = (red << info->var.red.offset) |
				(green << info->var.green.offset) |
				(blue << info->var.blue.offset) |
				(transp << info->var.transp.offset);

			pal[regno] = value;
			ret = 0;
		}
		break;
	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		break;
	}

	return ret;
}

static struct fb_fix_screeninfo seven_inch_lcd_fb_fix = {
	.id             = "lcd_drvr",
	.type           = FB_TYPE_PACKED_PIXELS,
	.visual         = FB_VISUAL_TRUECOLOR,
	.accel          = FB_ACCEL_NONE,
	.line_length    = 800*4,
};

static struct fb_var_screeninfo seven_inch_lcd_fb_var = {
	.width                  = 800,
	.height                 = 480,
	.bits_per_pixel     = 32,
	.xres                   = 800,
	.yres                   = 480,
	.xres_virtual        = 800,
	.yres_virtual       = 480,
	.activate               = FB_ACTIVATE_NOW,
	.vmode              = FB_VMODE_NONINTERLACED,
	.transp              = {24, 8, 0},
    .red                    = {16, 8, 0},
    .green              = {8, 8, 0},
    .blue               = {0, 8, 0},
};

int seven_inch_lcd_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	return vm_iomap_memory(vma, info->fix.smem_start, info->fix.smem_len);
}

static struct fb_ops seven_inch_lcd_fb_ops = {
	.owner          = THIS_MODULE,
	.fb_read        = fb_sys_read,
	.fb_write       = fb_sys_write,
	.fb_fillrect    = sys_fillrect,
	.fb_copyarea    = sys_copyarea,
	.fb_imageblit   = sys_imageblit,
	.fb_setcolreg   = seven_inch_lcd_fb_setcolreg,
	.fb_mmap        = seven_inch_lcd_fb_mmap,
};

static u64 platform_dma_mask = DMA_BIT_MASK(32);

static void set_dma_addr(dma_addr_t dma_addr)
{
	/* fpga2sdram interface has word address,
	 * but fpga2hps has byte address. */
		dma_addr = dma_addr / 4;

	/* Write address into FPGA-DMA */
	fpga_write_reg(1, dma_addr);
}

static int seven_inch_lcd_fb_probe(struct platform_device *pdev)
{
	struct fb_info *info;
	int ret;

	u32 vmem_size;
	unsigned char *vmem;

	dma_addr_t dma_addr;

	pdev->dev.dma_mask = &platform_dma_mask;
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	vmem_size = (seven_inch_lcd_fb_var.width * seven_inch_lcd_fb_var.height * seven_inch_lcd_fb_var.bits_per_pixel) / 8;

	printk(KERN_ERR "Allocating memory");
	vmem = dmam_alloc_coherent(&pdev->dev, vmem_size, &dma_addr, GFP_KERNEL);
	if (!vmem) {
		dev_err(&pdev->dev, "FB: dma_alloc_coherent error\n");
		printk(KERN_ERR "FB: dma_alloc_coherent error\n");
		return -ENOMEM;
	}
	printk(KERN_ERR "Allocating memory successful. Address: %d", (int)vmem);
	printk(KERN_ERR "DMA address: %d", (int)dma_addr);
	printk(KERN_ERR "Cleaning memory");
	memset(vmem, 0, vmem_size);

	printk(KERN_ERR "Allocating framebuffer");
	info = framebuffer_alloc(0, &pdev->dev);
	if (!info)
		return -ENOMEM;

	info->screen_base = vmem;
	info->fbops = &seven_inch_lcd_fb_ops;
	info->fix = seven_inch_lcd_fb_fix;
	info->fix.smem_start = dma_addr;
	info->fix.smem_len = vmem_size;
	info->var = seven_inch_lcd_fb_var;
	info->flags = FBINFO_DEFAULT;
	info->pseudo_palette = &seven_inch_lcd_fb_pseudo_palette;

	printk(KERN_ERR "Getting fpga regs address");
	/* Get FPGA registers address */
	fpga_regs = devm_ioremap(&pdev->dev, FPGA_REGS_BASE, REGSIZE);

	/* Disable refreshing */
	fpga_write_reg(0, 0);

	fpga_write_reg(2, 0x96a6a3cd);

	set_dma_addr(dma_addr);

	/* Enable refreshing */
	fpga_write_reg(0, 1);

	printk(KERN_ERR "Registering framebuffer");
	ret = register_framebuffer(info);
	if (ret < 0) {
		framebuffer_release(info);
		printk(KERN_ERR "Fail registering framebuffer");
		return ret;
	}

	printk(KERN_ERR "Setting platform_set_drvdata");
	platform_set_drvdata(pdev, info);

	return 0;
}

static int seven_inch_lcd_fb_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);

	fpga_write_reg(0, 0);

	if (info) {
		unregister_framebuffer(info);

		framebuffer_release(info);
	}

	return 0;
}

static const struct of_device_id seven_inch_lcd_of_match[] = {
	{ .compatible = "dev,custom-lcd", },
	{},
};

 MODULE_DEVICE_TABLE(of, seven_inch_lcd_of_match);

static struct platform_driver seven_inch_lcd_fb_driver = {
	.remove = seven_inch_lcd_fb_remove,
	.probe = seven_inch_lcd_fb_probe,
	.driver = {
		.name   = "lcd_drvr",
		 .owner  = THIS_MODULE,
		 .of_match_table = seven_inch_lcd_of_match
	},
};

//static struct platform_device *seven_inch_lcd_fb_device;

static int seven_inch_lcd_fb_init(void)
{
	int ret_val = 0;
	pr_info("Registering platform driver\n");

	ret_val = platform_driver_register(&seven_inch_lcd_fb_driver);
	if(ret_val != 0) {
        pr_err("Failed to probe seven_inch_lcd platform driver %d\n", ret_val);
        return ret_val;
    }
    pr_info("Success!\n");
    return 0;
}

static void seven_inch_lcd_fb_exit(void)
{
	pr_info("Custom LCD module exit\n");
	platform_driver_unregister(&seven_inch_lcd_fb_driver);
}

MODULE_AUTHOR("mishaskt");
MODULE_DESCRIPTION("seven_inch_lcd framebuffer driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

module_init(seven_inch_lcd_fb_init);
module_exit(seven_inch_lcd_fb_exit);
