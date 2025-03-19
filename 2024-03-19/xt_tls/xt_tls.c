#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/skbuff.h>
#include <linux/netfilter/x_tables.h>
#include <linux/netfilter_ipv4/ip_tables.h>
#include <linux/netfilter_ipv6/ip6_tables.h>
#include <linux/string.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/inet.h>
#include <asm/errno.h>

#include "xt_tls.h"
//#define XT_TLS_DEBUG 1

static int glob_match (const char *text, const char *pattern)
{
	do {
		/* Match single character or a '?' wildcard */
		if (*text == *pattern || *pattern == '?') {
			if (!*pattern++)
				return 0;  /* End of both strings: match */
		} else {
			/* Match single char against a '[' bracketed ']' pattern set */
			if (!*text || *pattern != '[')
				break;  /* Not a pattern set */
			while (*++pattern && *pattern != ']' && *text != *pattern) {
				if (*pattern == '-' && *(pattern - 1) != '[')
					if (*text > *(pattern - 1) && *text < *(pattern + 1)) {
						++pattern;
						break;
					}
			}
			if (!*pattern || *pattern == ']')
				return 1;  /* No match */
			while (*pattern && *pattern++ != ']');
		}
	} while (*++text && *pattern);

	/* Match any run of chars against a '*' wildcard */
	if (*pattern == '*') {
		if (!*++pattern)
			return 0;  /* Match: avoid recursion at end of pattern */
		/* Loop to handle additional pattern chars after the wildcard */
		while (*text) {
			if (glob_match(text, pattern) == 0)
				return 0;  /* Remainder matched */
			++text;  /* Absorb (match) this char and try again */
		}
	}
	if (!*text && !*pattern)
		return 0;  /* End of both strings: match */
	return 1;  /* No match */
}

/*
 * Searches through skb->data and looks for a
 * client or server handshake. A client
 * handshake is preferred as the SNI
 * field tells us what domain the client
 * wants to connect to.
 */
static int get_tls_hostname(const struct sk_buff *skb, char **dest, unsigned thoff)
{
	char *data, *tail;
	size_t data_len;
	u_int16_t tls_header_len;
	u_int8_t handshake_protocol;
	struct tcphdr _tcph;
	struct tcphdr *tcp_header = skb_header_pointer(skb, thoff, sizeof(_tcph), &_tcph);
	if (!tcp_header) {
	    return EPROTO;
	}

	// I'm not completely sure how this works (courtesy of StackOverflow), but it works
	data = (char *)((unsigned char *)tcp_header + (tcp_header->doff * 4));
	tail = (char *)skb_tail_pointer(skb);
	// Calculate packet data length
	data_len = (uintptr_t)tail - (uintptr_t)data;
//	printk("xt_tls: get_tls_hostname %u %u\n", data_len, thoff);

	// If this isn't an TLS handshake, abort
	if (data[0] != 0x16) {
//		printk("xt_tls: not a TLS handshake %u\n", data[0]);
		return EPROTO;
	}

	tls_header_len = (data[3] << 8) + data[4] + 5;
	handshake_protocol = data[5];

	// Even if we don't have all the data, try matching anyway
	if (tls_header_len > data_len)
		tls_header_len = data_len;

	if (tls_header_len > 4) {
		// Check only client hellos for now
		if (handshake_protocol == 0x01) {
			u_int offset, base_offset = 43, extension_offset = 2;
			u_int16_t session_id_len, cipher_len, compression_len, extensions_len;

			if (base_offset + 2 > data_len) {
#ifdef XT_TLS_DEBUG
				printk("xt_tls: Data length is to small (%d)\n", (int)data_len);
#endif
				return EPROTO;
			}

			// Get the length of the session ID
			session_id_len = data[base_offset];

#ifdef XT_TLS_DEBUG
			printk("xt_tls: Session ID length: %d\n", session_id_len);
#endif
			if ((session_id_len + base_offset + 2) > tls_header_len) {
#ifdef XT_TLS_DEBUG
				printk("xt_tls: TLS header length is smaller than session_id_len + base_offset +2 (%d > %d)\n", (session_id_len + base_offset + 2), tls_header_len);
#endif
				return EPROTO;
			}

			// Get the length of the ciphers
			memcpy(&cipher_len, &data[base_offset + session_id_len + 1], 2);
			cipher_len = ntohs(cipher_len);
			offset = base_offset + session_id_len + cipher_len + 2;
#ifdef XT_TLS_DEBUG
			printk("xt_tls: Cipher len: %d\n", cipher_len);
			printk("xt_tls: Offset (1): %d\n", offset);
#endif
			if (offset > tls_header_len) {
#ifdef XT_TLS_DEBUG
				printk("xt_tls: TLS header length is smaller than offset (%d > %d)\n", offset, tls_header_len);
#endif
				return EPROTO;
			}

			// Get the length of the compression types
			compression_len = data[offset + 1];
			offset += compression_len + 2;
#ifdef XT_TLS_DEBUG
			printk("xt_tls: Compression length: %d\n", compression_len);
			printk("xt_tls: Offset (2): %d\n", offset);
#endif
			if (offset > tls_header_len) {
#ifdef XT_TLS_DEBUG
				printk("xt_tls: TLS header length is smaller than offset w/compression (%d > %d)\n", offset, tls_header_len);
#endif
				return EPROTO;
			}

			// Get the length of all the extensions
			memcpy(&extensions_len, &data[offset], 2);
			extensions_len = ntohs(extensions_len);
#ifdef XT_TLS_DEBUG
			printk("xt_tls: Extensions length: %d\n", extensions_len);
#endif

			if ((extensions_len + offset) > tls_header_len) {
#ifdef XT_TLS_DEBUG
				printk("xt_tls: TLS header length is smaller than offset w/extensions (%d > %d)\n", (extensions_len + offset), tls_header_len);
#endif
				return EPROTO;
			}

			// Loop through all the extensions to find the SNI extension
			while (extension_offset < extensions_len)
			{
				u_int16_t extension_id, extension_len;

				memcpy(&extension_id, &data[offset + extension_offset], 2);
				extension_offset += 2;

				memcpy(&extension_len, &data[offset + extension_offset], 2);
				extension_offset += 2;

				extension_id = ntohs(extension_id), extension_len = ntohs(extension_len);

#ifdef XT_TLS_DEBUG
				printk("xt_tls: Extension ID: %d\n", extension_id);
				printk("xt_tls: Extension length: %d\n", extension_len);
#endif

				if (extension_id == 0) {
					u_int16_t name_length;
//					u_int16_t name_type;

					// We don't need the server name list length, so skip that
					extension_offset += 2;
					// We don't really need name_type at the moment
					// as there's only one type in the RFC-spec.
					// However I'm leaving it in here for
					// debugging purposes.
//					name_type = data[offset + extension_offset];
					extension_offset += 1;

					memcpy(&name_length, &data[offset + extension_offset], 2);
					name_length = ntohs(name_length);
					extension_offset += 2;

#ifdef XT_TLS_DEBUG
//					printk("xt_tls: Name type: %d\n", name_type);
					printk("xt_tls: Name length: %d\n", name_length);
#endif
					// Allocate an extra byte for the null-terminator
					*dest = kmalloc(name_length + 1, GFP_KERNEL);
					memcpy(*dest, &data[offset + extension_offset], name_length);
					// Make sure the string is always null-terminated.
					(*dest)[name_length] = 0;

					return 0;
				}

				extension_offset += extension_len;
			}
		}
	}

//	printk("tls_header_len: %u\n", tls_header_len);
	return EPROTO;
}

static bool tls_mt(const struct sk_buff *skb, struct xt_action_param *par)
{
	char *parsed_host;
	const struct xt_tls_info *info = par->matchinfo;
	int result;
	bool invert = (info->invert & XT_TLS_OP_HOST);
	bool match;

	if ((result = get_tls_hostname(skb, &parsed_host, par->thoff)) != 0)
		return false;

	match = !glob_match(parsed_host, info->tls_host);

#ifdef XT_TLS_DEBUG
	printk("xt_tls: domain:%s pattern:%s\n", parsed_host, info->tls_host);
	printk("xt_tls: Domain matches: %s, invert: %s\n", match ? "true" : "false", invert ? "true" : "false");
#endif
	if (invert)
		match = !match;

	kfree(parsed_host);

	return match;
}

static struct xt_match tls_mt_regs[] __read_mostly = {
	{
		.name       = "tls",
		.match      = tls_mt,
		.matchsize  = sizeof(struct xt_tls_info),
		.me         = THIS_MODULE,
	},
};

static int __init tls_mt_init (void)
{
	return xt_register_matches(tls_mt_regs, ARRAY_SIZE(tls_mt_regs));
}

static void __exit tls_mt_exit (void)
{
	xt_unregister_matches(tls_mt_regs, ARRAY_SIZE(tls_mt_regs));
}

module_init(tls_mt_init);
module_exit(tls_mt_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nils Andreas Svee <nils@stokkdalen.no>");
MODULE_DESCRIPTION("Xtables: TLS (SNI) matching");
MODULE_ALIAS("ipt_tls");
