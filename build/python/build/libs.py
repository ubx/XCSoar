from build.zlib import ZlibProject
from build.autotools import AutotoolsProject
from build.freetype import FreeTypeProject

zlib = ZlibProject(
    'http://zlib.net/zlib-1.2.11.tar.xz',
    'http://downloads.sourceforge.net/project/libpng/zlib/1.2.11/zlib-1.2.11.tar.xz',
    '4ff941449631ace0d4d203e3483be9dbc9da454084111f97ea0a2114e19bf066',
    'lib/libz.a',
)

freetype = FreeTypeProject(
    'http://download.savannah.gnu.org/releases/freetype/freetype-2.7.1.tar.bz2',
    'http://downloads.sourceforge.net/project/freetype/freetype2/2.7.1/freetype-2.7.1.tar.bz2',
    '3a3bb2c4e15ffb433f2032f50a5b5a92558206822e22bfe8cbe339af4aa82f88',
    'lib/libfreetype.a',
    [
        '--disable-shared', '--enable-static',
        '--without-bzip2', '--without-png',
        '--without-harfbuzz',
    ],
)

curl = AutotoolsProject(
    'http://curl.haxx.se/download/curl-7.54.0.tar.lzma',
    'http://github.com/curl/curl/releases/download/curl-7_54_0/curl-7.54.0.tar.lzma',
    'cd6aa6039f13e0b06e0a93e1b93754f6dc07f444812bb6c32be75a8f28c4070a',
    'lib/libcurl.a',
    [
        '--disable-shared', '--enable-static',
        '--disable-debug',
        '--enable-http',
        '--enable-ipv6',
        '--disable-ftp', '--disable-file',
        '--disable-ldap', '--disable-ldaps',
        '--disable-rtsp', '--disable-proxy', '--disable-dict', '--disable-telnet',
        '--disable-tftp', '--disable-pop3', '--disable-imap', '--disable-smb',
        '--disable-smtp',
        '--disable-gopher',
        '--disable-manual',
        '--disable-threaded-resolver', '--disable-verbose', '--disable-sspi',
        '--disable-crypto-auth', '--disable-ntlm-wb', '--disable-tls-srp', '--disable-cookies',
        '--without-ssl', '--without-gnutls', '--without-nss', '--without-libssh2',
    ],
    use_clang=True,
)

libpng = AutotoolsProject(
    'ftp://ftp.simplesystems.org/pub/libpng/png/src/libpng16/libpng-1.6.29.tar.xz',
    'http://downloads.sourceforge.net/project/libpng/libpng16/1.6.29/libpng-1.6.29.tar.xz',
    '4245b684e8fe829ebb76186327bb37ce5a639938b219882b53d64bd3cfc5f239',
    'lib/libpng.a',
    [
        '--disable-shared', '--enable-static',
        '--enable-arm-neon',
    ]
)

libjpeg = AutotoolsProject(
    'http://downloads.sourceforge.net/project/libjpeg-turbo/1.5.1/libjpeg-turbo-1.5.1.tar.gz',
    'http://sourceforge.mirrorservice.org/l/li/libjpeg-turbo/1.5.1/libjpeg-turbo-1.5.1.tar.gz',
    '41429d3d253017433f66e3d472b8c7d998491d2f41caa7306b8d9a6f2a2c666c',
    'lib/libjpeg.a',
    [
        '--disable-shared', '--enable-static',
        '--enable-arm-neon',
    ]
)
