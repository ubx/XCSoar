/*
 * Copyright (C) 2012-2017 Max Kellermann <max.kellermann@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * FOUNDATION OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef IPV4_ADDRESS_HXX
#define IPV4_ADDRESS_HXX

#include "SocketAddress.hxx"
#include "OS/ByteOrder.hpp"
#include "Compiler.h"

#include <stdint.h>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <netinet/in.h>
#endif

/**
 * An OO wrapper for struct sockaddr_in.
 */
class IPv4Address {
	struct sockaddr_in address;

#ifdef _WIN32
	static constexpr struct in_addr ConstructInAddr(uint8_t a, uint8_t b,
							uint8_t c, uint8_t d) noexcept {
		return {{{ a, b, c, d }}};
	}

	static constexpr struct in_addr ConstructInAddr(uint32_t x) noexcept {
		return ConstructInAddr(x >> 24, x >> 16, x >> 8, x);
	}
#else

#ifdef __BIONIC__
	typedef uint32_t in_addr_t;
#endif

	static constexpr in_addr_t ConstructInAddrT(uint8_t a, uint8_t b,
						    uint8_t c, uint8_t d) noexcept {
		return ToBE32((a << 24) | (b << 16) | (c << 8) | d);
	}

	static constexpr struct in_addr ConstructInAddr(uint32_t x) noexcept {
		return { ToBE32(x) };
	}

	static constexpr struct in_addr ConstructInAddr(uint8_t a, uint8_t b,
							uint8_t c, uint8_t d) noexcept {
		return { ConstructInAddrT(a, b, c, d) };
	}
#endif

	static constexpr struct sockaddr_in Construct(struct in_addr address,
						      uint16_t port) noexcept {
		return {
#if defined(__APPLE__)
			sizeof(struct sockaddr_in),
#endif
			AF_INET,
			ToBE16(port),
			address,
			{},
		};
	}

	static constexpr struct sockaddr_in Construct(uint32_t address,
						      uint16_t port) noexcept {
		return Construct(ConstructInAddr(address), port);
	}

public:
	IPv4Address() = default;

	constexpr IPv4Address(struct in_addr _address, uint16_t port) noexcept
		:address(Construct(_address, port)) {}

	constexpr IPv4Address(uint8_t a, uint8_t b, uint8_t c,
			      uint8_t d, uint16_t port) noexcept
		:IPv4Address(ConstructInAddr(a, b, c, d), port) {}

	constexpr explicit IPv4Address(uint16_t port) noexcept
		:IPv4Address(ConstructInAddr(INADDR_ANY), port) {}

	/**
	 * Convert a #SocketAddress to a #IPv4Address.  Its address family must be AF_INET.
	 */
	explicit IPv4Address(SocketAddress src) noexcept;

	constexpr operator SocketAddress() const noexcept {
		return SocketAddress(reinterpret_cast<const struct sockaddr *>(&address),
				     sizeof(address));
	}

	constexpr SocketAddress::size_type GetSize() const noexcept {
		return sizeof(address);
	}

	constexpr bool IsDefined() const noexcept {
		return address.sin_family != AF_UNSPEC;
	}

	constexpr uint16_t GetPort() const noexcept {
		return FromBE16(address.sin_port);
	}

	void SetPort(uint16_t port) noexcept {
		address.sin_port = ToBE16(port);
	}

	constexpr const struct in_addr &GetAddress() const noexcept {
		return address.sin_addr;
	}

#if !defined(_WIN32) && !defined(__BIONIC__)
	/**
	 * Returns a StaticSocketAddress for the specified device. Caller
	 * should check for validity of returned StaticSocketAddress.
	 *
	 * @param device is the device name f.i. "eth0"
	 * @return StaticSocketAddress, use IsDefined() to check valid result
	 */
	gcc_pure
	static IPv4Address GetDeviceAddress(const char *device) noexcept;

	/**
	 * Converts StaticSocketAddress to human readable string
	 *
	 * @param buffer is the result buffer
	 * @param buffer_size is the buffer size
	 * @return IP address on success, else nullptr
	 */
	gcc_pure
	const char *ToString(char *buffer, size_t buffer_size) const noexcept;
#endif
};

#endif
