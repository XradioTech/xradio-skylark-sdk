
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifndef _JPEG_ENC_H_
#define _JPEG_ENC_H_

#define DefaultDC       (1024)
#define ALIGN_8B(x)     (((x) + (7)) & ~(7))
#define ALIGN_16B(x)    (((x) + (15)) & ~(15))
#define ALIGN_32B(x)    (((x) + (31)) & ~(31))
#define ALIGN_4K(x)     (((x) + (4095)) & ~(4095))

#define NUM_QUANT_TBLS  2
#define DCTSIZE         8       /* The basic DCT block is 8x8 samples */
#define DCTSIZE2        64      /* DCTSIZE squared; # of elements in a block */

#define JpgYUV420	0
#define JpgYUV444	1
#define JpgYUV422	2

struct jpeg_ctl_ops {
	void (*writeHeader)(void *handle);
	int  (*setParameter)(void *handle, int indexType, void *param);
	void  (*setQuantTbl)(void *handle, int quality);
};

typedef struct JpegCtx {
	char     			*BaseAddr;
	unsigned short    	image_width;
	unsigned short    	image_height;
	unsigned short    	quant_tbl[2][DCTSIZE2 * 2];
	unsigned short    	quant_tbl_aw[DCTSIZE2 * 4]; /* modify to 256 word */
	int					dc_value[3];

	int 			  	JpgColorFormat;//0:420, 1:444, 2:422
	int               	quality;

	struct jpeg_ctl_ops *ctl_ops;
} JpegCtx;


typedef enum {
	VENC_IndexParamJpegQuality = 0,
	VENC_IndexParamJpegEncMode,
	VENC_IndexParamSetVsize,
	VENC_IndexParamSetHsize,

} VENC_IndexType;

JpegCtx *JpegEncCreate();

//JpegCtx *JpegCtx();

void JpegEncDestory(void *handle);


#endif /* _JPEG_ENC_H_ */

#ifdef __cplusplus
}
#endif /* __cplusplus */
