#include "asn1/DeviceIdentityBundle.h"
#include "asn1/DeviceIdentityInfo.h"

#include <boost/program_options.hpp>

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <array>
#include <algorithm>
#include <stdexcept>
#include <sstream>

namespace po = boost::program_options;

class DeviceIdentityInfoWrapper {
public:
    DeviceIdentityInfoWrapper(std::string const& essid, std::string const& password, std::string const& hostname,
            uint32_t device_id, uint16_t port)
        : m_device_id(device_id), m_port(port), m_pos(0)
    {
        std::memset(&m_info, 0, sizeof(m_info));

        std::copy(essid.begin(), essid.end(), m_essid.begin());
        std::copy(password.begin(), password.end(), m_password.begin());
        std::copy(hostname.begin(), hostname.end(), m_hostname.begin());

        m_info.wifiESSID.buf = m_essid.data();
        m_info.wifiESSID.size = essid.length();

        m_info.wifiPassword.buf = m_password.data();
        m_info.wifiPassword.size = password.length();

        m_info.targetHost.buf = m_hostname.data();
        m_info.targetHost.size = hostname.length();

        m_info.targetPort = port;
        m_info.deviceId = device_id;

        auto cb = [&](uint8_t const* buffer, size_t len) -> int {
            // Append to the output buffer stream
            if (len > 0 && m_pos + len < 4096) {
                std::copy(buffer, buffer + len, m_encoded.begin() + m_pos);
                m_pos += len;
                return 0;
            }

            return -1;
        };

        // TODO: Encode the certificate chain

        // Encode the DER blob
        asn_enc_rval_t ret = der_encode(&asn_DEF_DeviceIdentityInfo, reinterpret_cast<void*>(&m_info), [](void const* buffer, size_t size, void* callback_key) -> int {
                return (*static_cast<decltype(cb)*>(callback_key))(static_cast<uint8_t const*>(buffer), size);
            }, &cb);

        if (ret.encoded == -1) {
            int errnum = errno;
            std::stringstream ss;
            ss << "Failed to encode device identity info blob, failed type: " << ret.failed_type->name << " error: " << std::strerror(errnum);
            throw std::runtime_error(ss.str());
        }

        std::cout << "Length: " << m_pos << std::endl;
    }

    std::array<uint8_t, 4096> const& get_encoded(size_t& length) const {
        length = m_pos;
        return m_encoded;
    }

    virtual ~DeviceIdentityInfoWrapper() {
    }

private:
    std::array<uint8_t, 64> m_essid; //< The wireless network's ESSID
    std::array<uint8_t, 64> m_password; //< Password for the network (empty if none is required)
    std::array<uint8_t, 128> m_hostname; //< The hostname to connect to
    uint32_t m_device_id; //< The device ID to present
    uint16_t m_port; //< The port to connect to for the HERBIVORE service

    std::array<uint8_t, 4096> m_encoded;
    size_t m_pos;

    DeviceIdentityInfo_t m_info; //< The input unencoded bundle
};

class DeviceIdentityBundleWrapper {
    DeviceIdentityBundleWrapper(std::shared_ptr<DeviceIdentityInfoWrapper> wrapper, std::string const& signature) :
        m_pos(0)
    {
        std::memset(&m_bundle, 0, sizeof(m_bundle));

        auto cb = [&](uint8_t const* buffer, size_t len) -> int {
            // Append to the output buffer stream
            if (len > 0 && m_pos + len < 4096) {
                std::copy(buffer, buffer + len, m_encoded.begin() + m_pos);
                m_pos += len;
                return 0;
            }

            return -1;
        };

        // TODO: Encode the certificate chain

        // Encode the DER blob
        asn_enc_rval_t ret = der_encode(&asn_DEF_DeviceIdentityBundle, reinterpret_cast<void*>(&m_bundle), [](void const* buffer, size_t size, void* callback_key) -> int {
                return (*static_cast<decltype(cb)*>(callback_key))(static_cast<uint8_t const*>(buffer), size);
            }, &cb);

        if (ret.encoded == -1) {
            int errnum = errno;
            std::stringstream ss;
            ss << "Failed to encode device identity info bundle, failed type: " << ret.failed_type->name << " error: " << std::strerror(errnum);
            throw std::runtime_error(ss.str());
        }

        std::cout << "Bundle Length: " << m_pos << std::endl;

    }

    virtual ~DeviceIdentityBundleWrapper() {

    }

private:
    std::array<uint8_t, 32> m_r;
    std::array<uint8_t, 32> m_s;

    std::array<uint8_t, 4069> m_encoded;
    size_t m_pos;

    DeviceIdentityBundle_t m_bundle;
};

#if 0
static
std::shared_ptr<ECDSA<ECP, SHA1>::PrivateKey> load_private_key(std::string const& key_file)
{
    auto key = std::make_shared<ECDSA<ECP, SHA1>::PrivateKey>();
    std::string key_str;

    auto fs = std::make_unique<FileSource>(key_file.c_str(), true, new HexEncoder(new StringSink(key_str)));
    std::cout << key_str << std::endl;

//    try {
        key->Load(FileSource(key_file.c_str(), true).Ref());
//    } catch (CryptoPP::BERDecodeErr const& e) {
//        std::cerr << "Failure during key load: " << e.what() << std::endl;
//        throw;
//    }

    return key;
}

static
std::string gen_info_blob_signature(std::shared_ptr<ECDSA<ECP, SHA256>::PrivateKey> key, std::array<uint8_t, 4096> const& data, size_t len)
{
    AutoSeededRandomPool prng;
    std::string signature;
    signature.erase();

    ArraySource(data.data(), len, true,
        new SignerFilter(prng, ECDSA<ECP, SHA256>::Signer(*key),
            new StringSink(signature)
        )
    );

    return signature;
}
#endif

int main(int argc, char* argv[])
{
    po::options_description desc("Main Options");

    std::string essid,
                password,
                hostname,
                certs,
                signing_key;
    uint32_t device_id = 0;
    uint16_t port = 0;

    po::variables_map args;

    try {
        desc.add_options()
            ("help,h",              "Get some help (this screen)")
            ("essid,E",             po::value<std::string>(&essid)->required(),         "wireless network ESSID this device is to connect to")
            ("password,P",          po::value<std::string>(&password)->default_value(""),    "wireless network password (for WPA-PSK)")
            ("device-id,d",         po::value<uint32_t>(&device_id)->required(),   "device's identity number")
            ("hostname,H",          po::value<std::string>(&hostname)->required(),      "hostname to connect to at the backend")
            ("port,p",              po::value<uint16_t>(&port)->default_value(0),  "port the HERBIVORE service is running on")
            ("certs,C",             po::value<std::string>(&certs)->default_value(""),  "certificate chain from which this device's identity will be derived")
            ("signing-key,K",       po::value<std::string>(&signing_key)->required(),   "signing key, to sign the entire identity blob")
            ;

        po::store(po::command_line_parser(argc, argv)
                .options(desc)
                .run(), args);
        po::notify(args);
    } catch (po::error& e) {
        std::cout << "Error: " << e.what() << std::endl;
        std::cout << desc << std::endl;
        return EXIT_FAILURE;
    }

    if (args.count("help")) {
        // Print out help and exit
        std::cout << desc << std::endl;
        return EXIT_SUCCESS;
    }

    //std::cout << "Loading identity signing key..." << std::endl;
    //auto priv_key = load_private_key(signing_key);

    // Encode the connectivity information object
    std::cout << "Encoding identity information..." << std::endl;
    auto info = std::make_shared<DeviceIdentityInfoWrapper>(essid, password, hostname, device_id, port);

    // Generate the signature
    std::cout << "Signing identity information blob..." << std::endl;
    size_t encoded_len = 0;
    auto encoded_info = info->get_encoded(encoded_len);
    //auto signature = gen_info_blob_signature(priv_key, encoded_info, encoded_len);

    // Encode the blob

    return EXIT_SUCCESS;
}

