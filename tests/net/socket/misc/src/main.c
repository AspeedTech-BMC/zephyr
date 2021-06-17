/*
 * Copyright (c) 2019 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(net_test, CONFIG_NET_SOCKETS_LOG_LEVEL);

#include <stdio.h>
#include <ztest_assert.h>

#include <net/socket.h>
#include <net/dummy.h>

#include "../../socket_helpers.h"

void test_gethostname(void)
{
	static ZTEST_BMEM char buf[80];
	int res;

	res = gethostname(buf, sizeof(buf));
	zassert_equal(res, 0, "");
	printk("%s\n", buf);
	zassert_equal(strcmp(buf, "ztest_hostname"), 0, "");
}

void test_inet_pton(void)
{
	int res;
	uint8_t buf[32];

	res = inet_pton(AF_INET, "127.0.0.1", buf);
	zassert_equal(res, 1, "");

	res = inet_pton(AF_INET, "127.0.0.1a", buf);
	zassert_equal(res, 0, "");

	res = inet_pton(AF_INET6, "a:b:c:d:0:1:2:3", buf);
	zassert_equal(res, 1, "");

	res = inet_pton(AF_INET6, "::1", buf);
	zassert_equal(res, 1, "");

	res = inet_pton(AF_INET6, "1::", buf);
	zassert_equal(res, 1, "");

	res = inet_pton(AF_INET6, "a:b:c:d:0:1:2:3z", buf);
	zassert_equal(res, 0, "");
}

static struct in6_addr my_ipv6_addr1 = { { { 0x20, 0x01, 0x0d, 0xb8, 0, 0, 0, 0,
					     0, 0, 0, 0, 0, 0, 0, 0x1 } } };
static struct in_addr my_ipv4_addr1 = { { { 192, 0, 2, 1 } } };

static struct in6_addr my_ipv6_addr2 = { { { 0x20, 0x01, 0x0d, 0xb8, 0, 0, 0, 0,
					     0, 0, 0, 0, 0, 0, 0, 0x2 } } };
static struct in_addr my_ipv4_addr2 = { { { 192, 0, 2, 2 } } };

static uint8_t lladdr1[] = { 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 };
static uint8_t lladdr2[] = { 0x02, 0x02, 0x02, 0x02, 0x02, 0x02 };

struct dummy_context {
	struct net_if *iface;
	uint8_t *mac_address;
	struct in6_addr *ipv6_addr;
	struct in_addr *ipv4_addr;
};

static struct dummy_context dummy_data1 = {
	.mac_address = lladdr1,
	.ipv6_addr = &my_ipv6_addr1,
	.ipv4_addr = &my_ipv4_addr1
};
static struct dummy_context dummy_data2 = {
	.mac_address = lladdr2,
	.ipv6_addr = &my_ipv6_addr2,
	.ipv4_addr = &my_ipv4_addr2
};

const struct device *current_dev;
K_SEM_DEFINE(send_sem, 0, 1);

static int dummy_send(const struct device *dev, struct net_pkt *pkt)
{
	struct net_pkt *recv_pkt;
	int ret;

	ARG_UNUSED(dev);
	ARG_UNUSED(pkt);

	NET_DBG("Sending data (%d bytes) to iface %d\n",
		net_pkt_get_len(pkt), net_if_get_by_iface(net_pkt_iface(pkt)));

	current_dev = dev;

	k_sem_give(&send_sem);

	/* Report it back to the interface. */
	recv_pkt = net_pkt_clone(pkt, K_NO_WAIT);

	ret = net_recv_data(net_pkt_iface(recv_pkt), recv_pkt);
	zassert_equal(ret, 0, "Cannot receive data (%d)", ret);

	return 0;
}

static void dummy_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct dummy_context *ctx = dev->data;
	struct net_if_addr *ifaddr;

	ctx->iface = iface;

	net_if_set_link_addr(iface, ctx->mac_address, 6, NET_LINK_DUMMY);

	ifaddr = net_if_ipv6_addr_add(net_if_lookup_by_dev(dev), ctx->ipv6_addr,
				      NET_ADDR_MANUAL, 0);
	if (!ifaddr) {
		zassert_not_null(ifaddr, "ipv6 addr");
	}

	ifaddr = net_if_ipv4_addr_add(net_if_lookup_by_dev(dev), ctx->ipv4_addr,
				      NET_ADDR_MANUAL, 0);
	if (!ifaddr) {
		zassert_not_null(ifaddr, "ipv4 addr");
	}
}

static struct dummy_api dummy_api_funcs = {
	.iface_api.init = dummy_iface_init,
	.send = dummy_send,
};

static int dummy_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

#define DEV1_NAME "dummy_1"
#define DEV2_NAME "dummy_2"

NET_DEVICE_INIT(dummy_1, DEV1_NAME, dummy_init,
		NULL, &dummy_data1, NULL,
		CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &dummy_api_funcs,
		DUMMY_L2, NET_L2_GET_CTX_TYPE(DUMMY_L2), 127);


NET_DEVICE_INIT(dummy_2, DEV2_NAME, dummy_init,
		NULL, &dummy_data2, NULL,
		CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &dummy_api_funcs,
		DUMMY_L2, NET_L2_GET_CTX_TYPE(DUMMY_L2), 127);

#define DST_PORT 4242
#define BIND_PORT 4240

void test_so_bindtodevice(int sock_c, int sock_s, struct sockaddr *peer_addr,
			  socklen_t peer_addrlen, struct sockaddr *bind_addr,
			  socklen_t bind_addrlen)
{
	int ret;
	struct ifreq ifreq = { 0 };
	const struct device *dev1 = device_get_binding(DEV1_NAME);
	const struct device *dev2 = device_get_binding(DEV2_NAME);

	uint8_t send_buf[32];
	uint8_t recv_buf[sizeof(send_buf)] = { 0 };

	ret = bind(sock_s, bind_addr, bind_addrlen);
	zassert_equal(ret, 0, "bind failed, %d", errno);

	/* Bind server socket with interface 2. */

	strcpy(ifreq.ifr_name, dev2->name);
	ret = setsockopt(sock_s, SOL_SOCKET, SO_BINDTODEVICE, &ifreq,
			 sizeof(ifreq));
	zassert_equal(ret, 0, "SO_BINDTODEVICE failed, %d", errno);

	/* Bind client socket with interface 1 and send a packet. */

	current_dev = NULL;
	k_sem_reset(&send_sem);
	strcpy(ifreq.ifr_name, dev1->name);
	strcpy(send_buf, dev1->name);

	ret = setsockopt(sock_c, SOL_SOCKET, SO_BINDTODEVICE, &ifreq,
			 sizeof(ifreq));
	zassert_equal(ret, 0, "SO_BINDTODEVICE failed, %d", errno);

	ret = sendto(sock_c, send_buf, strlen(send_buf) + 1, 0,
		     peer_addr, peer_addrlen);
	zassert_equal(ret, strlen(send_buf) + 1, "sendto failed, %d", errno);

	ret = k_sem_take(&send_sem, K_MSEC(100));
	zassert_equal(ret, 0, "iface did not receive packet");

	zassert_equal_ptr(dev1, current_dev, "invalid interface used");

	k_msleep(10);

	/* Bind client socket with interface 2 and send a packet. */

	current_dev = NULL;
	k_sem_reset(&send_sem);
	strcpy(ifreq.ifr_name, dev2->name);
	strcpy(send_buf, dev2->name);

	ret = setsockopt(sock_c, SOL_SOCKET, SO_BINDTODEVICE, &ifreq,
			 sizeof(ifreq));
	zassert_equal(ret, 0, "SO_BINDTODEVICE failed, %d", errno);

	ret = sendto(sock_c, send_buf, strlen(send_buf) + 1, 0,
		     peer_addr, peer_addrlen);
	zassert_equal(ret, strlen(send_buf) + 1, "sendto failed, %d", errno);

	ret = k_sem_take(&send_sem, K_MSEC(100));
	zassert_equal(ret, 0, "iface did not receive packet");

	zassert_equal_ptr(dev2, current_dev, "invalid interface used");

	/* Server socket should only receive data from the bound interface. */

	k_msleep(10);

	ret = recv(sock_s, recv_buf, sizeof(recv_buf), MSG_DONTWAIT);
	zassert_true(ret > 0, "recv failed, %d", errno);
	zassert_mem_equal(recv_buf, dev2->name, strlen(dev2->name),
			 "received datagram from invalid interface");

	/* Remove the binding from the server socket. */

	strcpy(ifreq.ifr_name, "");
	ret = setsockopt(sock_s, SOL_SOCKET, SO_BINDTODEVICE, &ifreq,
			 sizeof(ifreq));
	zassert_equal(ret, 0, "SO_BINDTODEVICE failed, %d", errno);

	/* Bind client socket with interface 1 again. */

	k_sem_reset(&send_sem);
	strcpy(ifreq.ifr_name, dev1->name);
	strcpy(send_buf, dev1->name);

	ret = setsockopt(sock_c, SOL_SOCKET, SO_BINDTODEVICE, &ifreq,
			 sizeof(ifreq));
	zassert_equal(ret, 0, "SO_BINDTODEVICE failed, %d", errno);

	ret = sendto(sock_c, send_buf, strlen(send_buf) + 1, 0,
		     peer_addr, peer_addrlen);
	zassert_equal(ret, strlen(send_buf) + 1, "sendto failed, %d", errno);

	ret = k_sem_take(&send_sem, K_MSEC(100));
	zassert_equal(ret, 0, "iface did not receive packet");

	zassert_equal_ptr(dev1, current_dev, "invalid interface used");

	/* Server socket should now receive data from iterface 1 as well. */

	k_msleep(10);

	ret = recv(sock_s, recv_buf, sizeof(recv_buf), MSG_DONTWAIT);
	zassert_true(ret > 0, "recv failed, %d", errno);
	zassert_mem_equal(recv_buf, dev1->name, strlen(dev1->name),
			 "received datagram from invalid interface");

	ret = close(sock_c);
	zassert_equal(ret, 0, "close failed, %d", errno);
	ret = close(sock_s);
	zassert_equal(ret, 0, "close failed, %d", errno);
}

void test_ipv4_so_bindtodevice(void)
{
	int ret;
	int sock_c;
	int sock_s;
	struct sockaddr_in peer_addr;
	struct sockaddr_in bind_addr = {
		.sin_family = AF_INET,
		.sin_port = htons(DST_PORT),
		.sin_addr = INADDR_ANY_INIT,
	};

	sock_c = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	zassert_true(sock_c >= 0, "socket open failed");
	sock_s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	zassert_true(sock_s >= 0, "socket open failed");

	peer_addr.sin_family = AF_INET;
	peer_addr.sin_port = htons(DST_PORT);
	ret = inet_pton(AF_INET, CONFIG_NET_CONFIG_PEER_IPV4_ADDR,
			&peer_addr.sin_addr);
	zassert_equal(ret, 1, "inet_pton failed");

	test_so_bindtodevice(sock_c, sock_s, (struct sockaddr *)&peer_addr,
			     sizeof(peer_addr), (struct sockaddr *)&bind_addr,
			     sizeof(bind_addr));
}

void test_ipv6_so_bindtodevice(void)
{
	int ret;
	int sock_c;
	int sock_s;
	struct sockaddr_in6 peer_addr;
	struct sockaddr_in6 bind_addr = {
		.sin6_family = AF_INET6,
		.sin6_port = htons(DST_PORT),
		.sin6_addr = IN6ADDR_ANY_INIT,
	};

	sock_c = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
	zassert_true(sock_c >= 0, "socket open failed");
	sock_s = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
	zassert_true(sock_s >= 0, "socket open failed");

	peer_addr.sin6_family = AF_INET6;
	peer_addr.sin6_port = htons(DST_PORT);
	ret = inet_pton(AF_INET6, CONFIG_NET_CONFIG_PEER_IPV6_ADDR,
			&peer_addr.sin6_addr);
	zassert_equal(ret, 1, "inet_pton failed");

	test_so_bindtodevice(sock_c, sock_s, (struct sockaddr *)&peer_addr,
			     sizeof(peer_addr), (struct sockaddr *)&bind_addr,
			     sizeof(bind_addr));
}

void test_main(void)
{
	k_thread_system_pool_assign(k_current_get());

	ztest_test_suite(socket_misc,
			 ztest_user_unit_test(test_gethostname),
			 ztest_user_unit_test(test_inet_pton),
			 ztest_unit_test(test_ipv4_so_bindtodevice),
			 ztest_unit_test(test_ipv6_so_bindtodevice));

	ztest_run_test_suite(socket_misc);
}
