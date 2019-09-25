/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#if (__CONFIG_MBUF_IMPL_MODE == 0)

#include "sys/mbuf_0.h"
#include "mbuf_util.h"

#define MBUF_SIZE       sizeof(struct mbuf) /* (24 + 24) == 48 */

#if MBUF_OPT_LIMIT_MEM

#define MBUF_LIMIT_MEM_DBG_ON   0

#include "kernel/os/os_thread.h"

/* limitation of memory usage
 *   - MBUF_TX_MEM_MAX: max sum of tx mem, 0 for no limit
 *   - MBUF_RX_MEM_MAX: max sum of rx mem, 0 for no limit
 *   - MBUF_TXRX_MEM_MAX: max sum of tx and rx mem, 0 for no limit.
 *                        MUST less than (MBUF_TX_MEM_MAX + MBUF_RX_MEM_MAX)
 */
uint32_t MBUF_TX_MEM_MAX   = (20 * 1024);
uint32_t MBUF_RX_MEM_MAX   = (20 * 1024);
uint32_t MBUF_TXRX_MEM_MAX = (32 * 1024);

static int32_t m_tx_mem_sum = 0;
static int32_t m_rx_mem_sum = 0;
static int32_t m_txrx_mem_sum = 0;

static __inline void mb_mem_lock(void)
{
	OS_ThreadSuspendScheduler();
}

static __inline void mb_mem_unlock(void)
{
	OS_ThreadResumeScheduler();
}

#if MBUF_LIMIT_MEM_DBG_ON
static int32_t m_tx_mem_sum_max = 0;
static int32_t m_rx_mem_sum_max = 0;
static int32_t m_txrx_mem_sum_max = 0;

void mb_limit_mem_info(void)
{
#define MB_SUM_VAL(v)	(v), (v) / 1024
	MBUF_LOG(1, "<<< mbuf limit mem info >>>\n"
	            "m_tx_mem_sum   %5u (%2u KB), max %5u (%2u KB)\n"
	            "m_rx_mem_sum   %5u (%2u KB), max %5u (%2u KB)\n"
	            "m_txrx_mem_sum %5u (%2u KB), max %5u (%2u KB)\n",
	         MB_SUM_VAL(m_tx_mem_sum), MB_SUM_VAL(m_tx_mem_sum_max),
	         MB_SUM_VAL(m_rx_mem_sum), MB_SUM_VAL(m_rx_mem_sum_max),
	         MB_SUM_VAL(m_txrx_mem_sum), MB_SUM_VAL(m_txrx_mem_sum_max));

	if (m_tx_mem_sum + m_rx_mem_sum != m_txrx_mem_sum) {
		MBUF_ERR("tx %d + rx %d != total %d\n",
		         m_tx_mem_sum, m_rx_mem_sum, m_txrx_mem_sum);
	}

#undef MB_SUM_VAL
}
#endif /* MBUF_LIMIT_MEM_DBG_ON */

static __inline int mb_limit_mem_inc(uint8_t flag, int32_t len)
{
	int32_t sum;

	mb_mem_lock();
	sum = m_txrx_mem_sum + len;
	if ((MBUF_TXRX_MEM_MAX > 0) && (sum > MBUF_TXRX_MEM_MAX)) {
		mb_mem_unlock();
		MBUF_DBG("txrx mem sum %d + len %d > %d, flag %#x\n",
				 m_txrx_mem_sum, len, MBUF_TXRX_MEM_MAX, flag);
		return -1;
	}
	if (flag & MBUF_GET_FLAG_LIMIT_TX) {
		sum = m_tx_mem_sum + len;
		if ((MBUF_TX_MEM_MAX > 0) && (sum > MBUF_TX_MEM_MAX)) {
			mb_mem_unlock();
			MBUF_DBG("tx mem sum %d + len %d > %d, flag %#x\n",
					 m_tx_mem_sum, len, MBUF_TX_MEM_MAX, flag);
			return -1;
		} else {
			m_tx_mem_sum = sum;
#if MBUF_LIMIT_MEM_DBG_ON
			if (m_tx_mem_sum > m_tx_mem_sum_max) {
				m_tx_mem_sum_max = m_tx_mem_sum;
			}
#endif
		}
	}
	if (flag & MBUF_GET_FLAG_LIMIT_RX) {
		sum = m_rx_mem_sum + len;
		if ((MBUF_RX_MEM_MAX > 0) && (sum > MBUF_RX_MEM_MAX)) {
			mb_mem_unlock();
			MBUF_DBG("rx mem sum %d + len %d > %d, flag %#x\n",
					 m_rx_mem_sum, len, MBUF_RX_MEM_MAX, flag);
			return -1;
		} else {
			m_rx_mem_sum = sum;
#if MBUF_LIMIT_MEM_DBG_ON
			if (m_rx_mem_sum > m_rx_mem_sum_max) {
				m_rx_mem_sum_max = m_rx_mem_sum;
			}
#endif
		}
	}
	m_txrx_mem_sum += len;
#if MBUF_LIMIT_MEM_DBG_ON
	if (m_txrx_mem_sum > m_txrx_mem_sum_max) {
		m_txrx_mem_sum_max = m_txrx_mem_sum;
	}
#endif
	mb_mem_unlock();
	return 0;
}

static void mb_limit_mem_dec(uint8_t flag, int32_t len)
{
	mb_mem_lock();

	if (m_txrx_mem_sum < len) {
		MBUF_ERR("m_txrx_mem_sum %d < len %d\n", m_txrx_mem_sum, len);
	}
	m_txrx_mem_sum -= len;

	if (flag & MBUF_GET_FLAG_LIMIT_TX) {
		if (m_tx_mem_sum < len) {
			MBUF_ERR("m_tx_mem_sum %d < len %d\n", m_tx_mem_sum, len);
		}
		m_tx_mem_sum -= len;
	}

	if (flag & MBUF_GET_FLAG_LIMIT_RX) {
		if (m_rx_mem_sum < len) {
			MBUF_ERR("m_rx_mem_sum %d < len %d\n", m_rx_mem_sum, len);
		}
		m_rx_mem_sum -= len;
	}

	mb_mem_unlock();
}

void mb_mem_set_limit(uint32_t tx, uint32_t rx, uint32_t txrx)
{
	MBUF_TX_MEM_MAX = tx;
	MBUF_RX_MEM_MAX = rx;
	MBUF_TXRX_MEM_MAX = txrx;
}

void mb_mem_get_limit(uint32_t *tx, uint32_t *rx, uint32_t *txrx)
{
	 *tx = MBUF_TX_MEM_MAX;
	 *rx = MBUF_RX_MEM_MAX;
	 *txrx = MBUF_TXRX_MEM_MAX;
}

#endif /* MBUF_OPT_LIMIT_MEM */

/*
 * @param tx
 *   - 1 means mbuf is used to do TX, reserve head/tail space
 *   - 0 means mbuf is used to do RX, no need to reserve head/tail space
 *   - if MBUF_OPT_LIMIT_MEM == 1, bitwise OR with MBUF_GET_FLAG_XXX
 * @return a mbuf including @len data
 */
struct mbuf *mb_get(int len, int tx)
{
	if (len < 0) {
		MBUF_ERR("len %d\n", len);
		return NULL;
	}

#if MBUF_OPT_LIMIT_MEM
	uint8_t flag = tx & MBUF_GET_FLAG_MASK;
	tx &= 0x1;
#endif

	int32_t tot_len = MBUF_SIZE + len;
	if (tx) {
		tot_len += MBUF_HEAD_SPACE + MBUF_TAIL_SPACE;
	}

#if MBUF_OPT_LIMIT_MEM
	if (flag && mb_limit_mem_inc(flag, tot_len) != 0) {
		return NULL;
	}
#endif /* MBUF_OPT_LIMIT_MEM */

	struct mbuf *m = (struct mbuf *)MB_MALLOC(tot_len);
	if (m) {
		MB_MEMSET(m, 0, MBUF_SIZE);
		m->m_buf = (uint8_t *)m + MBUF_SIZE;
		m->m_data = m->m_buf;
		m->m_len = len;
		if (tx) {
			m->m_data += MBUF_HEAD_SPACE;
			m->m_headspace = MBUF_HEAD_SPACE;
			m->m_tailspace = MBUF_TAIL_SPACE;
		}
		m->m_flags = M_PKTHDR;
		m->m_pkthdr.len = len;
#if MBUF_OPT_LIMIT_MEM
		m->m_type = flag;
#endif
	} else {
#if MBUF_OPT_LIMIT_MEM
		if (flag) {
			mb_limit_mem_dec(flag, tot_len);
		}
		MBUF_WRN("MB_MALLOC() fail, len %d\n", tot_len);
#else
		MBUF_DBG("MB_MALLOC() fail, len %d\n", tot_len);
#endif
	}

	return m;
}

/*
 * Free a mbuf.
 */
void mb_free(struct mbuf *m)
{
	if (m == NULL) {
		MBUF_WRN("mb_free(), m is NULL\n");
		return;
	}

#if MBUF_OPT_LIMIT_MEM
	uint8_t flag = m->m_type & MBUF_GET_FLAG_MASK;
	if (flag) {
		int32_t len = MBUF_SIZE + m->m_len + m->m_headspace + m->m_tailspace;
		mb_limit_mem_dec(flag, len);
	}
#endif /* MBUF_OPT_LIMIT_MEM */

	MB_FREE(m);
}

/*
 * Create a new mbuf including all data
 */
static __inline struct mbuf *mb_create(uint8_t *data, int len, int tx)
{
	struct mbuf *m = mb_get(len, tx);

	if (m && data) {
		MB_MEMCPY(m->m_data, data, len);
	}
	return m;
}

/* Add space at the head of mbuf, no sanity checks */
static void mb_adj_head(struct mbuf *m, int increment)
{
	m->m_headspace -= increment;
	m->m_data -= increment;
	m->m_len += increment;
	m->m_pkthdr.len += increment;
}

/* Add space at the tail of mbuf, no sanity checks */
static void mb_adj_tail(struct mbuf *m, int increment)
{
	m->m_tailspace -= increment;
	m->m_len += increment;
	m->m_pkthdr.len += increment;
}

/* copy some members of mbuf from @s to @d */
static void mb_pkthdr_init(struct mbuf *d, struct mbuf *s, int32_t pktlen)
{
	d->m_flags = (s->m_flags & M_COPYFLAGS) | M_PKTHDR;
	MB_MEMCPY(&d->m_pkthdr, &s->m_pkthdr, sizeof(struct pkthdr));
	d->m_pkthdr.len = pktlen;
}

/*
 * Trim data from head or tail.
 *
 * @return 0 on success, -1 on failure.
 */
int mb_adj(struct mbuf *m, int req_len)
{
	if (req_len >= 0) {
		/* Trim from head. */
		if (req_len > m->m_len) {
			MBUF_ERR("trim from head failed, %d > %d\n", req_len, (int)m->m_len);
			return -1; /* no enough data to trim */
		}
		mb_adj_head(m, -req_len);
	} else {
		/* Trim from tail. */
		req_len = -req_len;
		if (req_len > m->m_len) {
			MBUF_ERR("trim from tail failed, %d > %d\n", req_len, (int)m->m_len);
			return -1; /* no enough data to trim */
		}
		mb_adj_tail(m, -req_len);
	}
	return 0;
}

/*
 * Copy data from an mbuf chain starting "off" bytes from the beginning,
 * continuing for "len" bytes, into the indicated buffer.
 *
 * @return the number of bytes copied
 */
int mb_copydata(const struct mbuf *m, int off, int len, uint8_t *cp)
{
	if (off < 0 || len <= 0) {
		MBUF_ERR("off %d, len %d\n", off, len);
		return 0;
	}

	int copy_len = m->m_len - off;
	if (copy_len > len)
		copy_len = len;
	MB_MEMCPY(cp, m->m_data + off, copy_len);
	return copy_len;
}

/*
 * Copy a packet header mbuf chain into a completely new chain.
 */
struct mbuf *mb_dup(struct mbuf *m)
{
	if (m == NULL) {
		MBUF_ERR("m is NULL\n");
		return NULL;
	}

	int32_t headspace = m->m_headspace;
	int32_t tailspace = m->m_tailspace;
	struct mbuf *nm = mb_get(m->m_len + headspace + tailspace,
	                         0 | MBUF_GET_FLAG_LIMIT_RX);
	if (nm == NULL) {
		return NULL;
	}

	mb_adj_head(nm, -headspace);
	mb_adj_tail(nm, -tailspace);
	MB_MEMCPY(nm->m_data, m->m_data, m->m_len);
	mb_pkthdr_init(nm, m, m->m_len);
	return nm;
}

/*
 * Rearange an mbuf chain so that len bytes are contiguous
 * and in the data area of an mbuf (so that mtod will work
 * for a structure of size len).  Returns the resulting
 * mbuf chain on success, frees it and returns null on failure.
 * If there is room, it will add up to max_protohdr-len extra bytes to the
 * contiguous region in an attempt to avoid being called next time.
 */
struct mbuf *mb_pullup(struct mbuf *m, int len) // NOT really support!
{
	if (m->m_len < len) {
		mb_free(m);
		return NULL;
	}
	return m;
}

/*
 * Partition an mbuf chain in two pieces, returning the tail --
 * all but the first len0 bytes.  In case of failure, it returns NULL and
 * attempts to restore the chain to its original state.
 *
 * Note that the resulting mbufs might be read-only, because the new
 * mbuf can end up sharing an mbuf cluster with the original mbuf if
 * the "breaking point" happens to lie within a cluster mbuf. Use the
 * M_WRITABLE() macro to check for this case.
 */
struct mbuf *mb_split(struct mbuf *m0, int len0)
{
	if (m0 == NULL || len0 <= 0) {
		MBUF_ERR("m0 %p, len0 %d\n", m0, len0);
		return NULL;
	}

	if (m0->m_len <= len0) {
		MBUF_ERR("m0->m_len %d < len0 %d\n", m0->m_len, len0);
		return NULL;
	}

	/* create a new mbuf to save all the tail data */
	int len = m0->m_len - len0;
	struct mbuf *m = mb_create(m0->m_data + len0, len,
	                           0 | MBUF_GET_FLAG_LIMIT_RX); /* for RX only */
	if (m == NULL) {
		return NULL;
	}

	mb_pkthdr_init(m, m0, len);
	mb_adj_tail(m0, -len); /* adjust @m0, its length is len0 */
	return m;
}

/*
 * Append the specified data to the indicated mbuf chain,
 * Extend the mbuf chain if the new data does not fit in
 * existing space.
 *
 * Return 1 if able to complete the job; otherwise 0.
 */
int mb_append(struct mbuf *m, int len, const uint8_t *cp)
{
	if (len > m->m_tailspace) {
		MBUF_ERR("%d > %d\n", len, (int)m->m_tailspace);
		return 0;
	}

	uint8_t *dst = m->m_data + m->m_len;
	mb_adj_tail(m, len);
	if (cp) {
		MB_MEMCPY(dst, cp, len);
	}
	return 1;
}

/*
 * Adjust the mbuf to reserve space directly.
 *
 * @return 0 on success, -1 on failure.
 */
int mb_reserve(struct mbuf *m, int len, uint16_t headspace, uint16_t tailspace)
{
	uint8_t *buf_end = m->m_data + m->m_len + m->m_tailspace;
	int buf_len = m->m_headspace + m->m_len + m->m_tailspace;

	if (buf_len < headspace + len + tailspace) {
		MBUF_ERR("(%d + %d + %d) < (%d + %d + %d)\n",
		         m->m_headspace, m->m_len, m->m_tailspace,
		         headspace, len, tailspace);
		return -1;
	}

	m->m_tailspace = tailspace;
	m->m_len = len;
	m->m_pkthdr.len = len;
	m->m_data = buf_end - tailspace - len;
	m->m_headspace = buf_len - tailspace - len;
	return 0;
}

#endif /* (__CONFIG_MBUF_IMPL_MODE == 0) */
