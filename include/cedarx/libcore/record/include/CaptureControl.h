#ifndef CAPTURE_CONTROL_H
#define CAPTURE_CONTROL_H
#include "cdx_log.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int                      nChannels;
    int                      nSamplerate;
    int                      nBitpersample;
} CdxCapbkCfg;

typedef struct CaptureCtrl CaptureCtrl;

struct CaptureControlOpsS
{
    void (*cdxDestroy)(CaptureCtrl* c);
    void (*cdxSetFormat)(CaptureCtrl* c, CdxCapbkCfg* cfg);
    int (*cdxStart)(CaptureCtrl* c);
    int (*cdxStop)(CaptureCtrl* c);
    int (*cdxRead)(CaptureCtrl* c, void* pData, int nDataSize);
};

struct CaptureCtrl
{
    const struct CaptureControlOpsS* ops;
};

CaptureCtrl* CaptureDeviceCreate();

static inline void CaptureDeviceDestroy(CaptureCtrl* c)
{
    CDX_CHECK(c);
    CDX_CHECK(c->ops);
    CDX_CHECK(c->ops->cdxDestroy);
    return c->ops->cdxDestroy(c);
}

static inline void CaptureDeviceSetFormat(CaptureCtrl* c, CdxCapbkCfg* cfg)
{
    CDX_CHECK(c);
    CDX_CHECK(c->ops);
    CDX_CHECK(c->ops->cdxSetFormat);
    return c->ops->cdxSetFormat(c, cfg);
}

static inline int CaptureDeviceStart(CaptureCtrl* c)
{
    CDX_CHECK(c);
    CDX_CHECK(c->ops);
    CDX_CHECK(c->ops->cdxStart);
    return c->ops->cdxStart(c);
}

static inline int CaptureDeviceStop(CaptureCtrl* c)
{
    CDX_CHECK(c);
    CDX_CHECK(c->ops);
    CDX_CHECK(c->ops->cdxStop);
    return c->ops->cdxStop(c);
}

static inline int CaptureDeviceRead(CaptureCtrl* c, void* pData, int nDataSize)
{
    CDX_CHECK(c);
    CDX_CHECK(c->ops);
    CDX_CHECK(c->ops->cdxRead);
    return c->ops->cdxRead(c, pData, nDataSize);
}

#ifdef __cplusplus
}
#endif

#endif
