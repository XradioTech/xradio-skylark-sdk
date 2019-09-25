/*
 * Copyright (c) 2008-2016 Allwinner Technology Co. Ltd.
 * All rights reserved.
 *
 * File : CdxWriter.h
 * Description : Allwinner Muxer Writer Definition
 * History :
 *
 */

#ifndef __CDX_WRITER_H__
#define __CDX_WRITER_H__

#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cdx_log.h>

typedef struct CdxWriterCreatorS CdxWriterCreatorT;
typedef struct CdxWriterS CdxWriterT;

struct CdxWriterOps{
    int (*cdxConnect)(CdxWriterT *writer);
    int (*cdxRead)(CdxWriterT *writer, void *buf, int size);
    int (*cdxWrite)(CdxWriterT *writer, void *buf, int size);
    long (*cdxSeek)(CdxWriterT *writer, long moffset, int mwhere);
    long (*cdxTell)(CdxWriterT *writer);
    int  (*cdxClose)(CdxWriterT *writer);
};

struct CdxWriterS
{
    const struct CdxWriterOps* ops;
};

struct CdxWriterCreatorS
{
    CdxWriterT *(*create)(char *);
};

static inline int CdxWriterConnect(CdxWriterT *w)
{
    CDX_CHECK(w);
    CDX_CHECK(w->ops);
    CDX_CHECK(w->ops->cdxConnect);
    return w->ops->cdxConnect(w);
}

static inline int CdxWriterRead(CdxWriterT *w, void *buf, int size)
{
    CDX_CHECK(w);
    CDX_CHECK(w->ops);
    CDX_CHECK(w->ops->cdxRead);
    return w->ops->cdxRead(w, buf, size);
}

static inline int CdxWriterWrite(CdxWriterT *w, void *buf, int size)
{
    CDX_CHECK(w);
    CDX_CHECK(w->ops);
    CDX_CHECK(w->ops->cdxWrite);
    return w->ops->cdxWrite(w, buf, size);
}

static inline int CdxWriterSeek(CdxWriterT *w, long moffset, int mwhere)
{
    CDX_CHECK(w);
    CDX_CHECK(w->ops);
    CDX_CHECK(w->ops->cdxSeek);
    return w->ops->cdxSeek(w, moffset, mwhere);
}

static inline int CdxWriterTell(CdxWriterT *w)
{
    CDX_CHECK(w);
    CDX_CHECK(w->ops);
    CDX_CHECK(w->ops->cdxTell);
    return w->ops->cdxTell(w);
}

static inline int CdxWriterClose(CdxWriterT *w)
{
    CDX_CHECK(w);
    CDX_CHECK(w->ops);
    CDX_CHECK(w->ops->cdxClose);
    return w->ops->cdxClose(w);
}

int CdxWriterOpen(char *url, CdxWriterT **writer);

#endif
