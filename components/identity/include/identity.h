#pragma once

#include <stdint.h>
#include <stddef.h>

#define IDENTITY_BLOB_LENGTH_MAX            4096
#define IDENTITY_KEY_LENGTH_MAX             128
#define IDENTITY_ESSID_LEN_MAX              64
#define IDENTITY_PASSWORD_LEN_MAX           64
#define IDENTITY_TARGET_HOST_LEN_MAX        128

/**
 * Structure representing the decoded and verified identity for this device.
 */
struct identity {
    /**
     * The 802.11 AP ESSID we are to connect to
     */
    char wifi_essid[IDENTITY_ESSID_LEN_MAX];

    /**
     * The 802.11 AP's pre-shared key (for WPA2)
     */
    char wifi_password[IDENTITY_PASSWORD_LEN_MAX];

    /**
     * The target host name, to be resolved using DNS
     */
    char target_host[IDENTITY_TARGET_HOST_LEN_MAX];

    /**
     * Then port on the target host
     */
    uint16_t target_port;

    /**
     * The device identifier, to be included in messages to the backend
     */
    uint32_t device_id;
};

/**
 * Load the device's identity, and read it into an easy-to-use structure after
 * validating the signature and other important details. This is represented by
 * the ASN.1 compound structure DeviceIdentityBundle.
 *
 * \param pident The identity struct to be populated (see above)
 * \param bundle The raw bundle, encoded as ASN.1 DER
 * \param bundle_length The length of the bundle, in bytes
 *
 * \return 0 on success, an error code otherwise.
 */
int identity_read(struct identity *pident, void *bundle, size_t bundle_length);

/**
 * Given the device's identity key, generate a CSR representing that identity.
 *
 * \param ident_key The identity private key, serialized as DER
 * \param ident_key_len The length of the identity key as serialized
 * \param ident_csr The identity CSR, stored as PEM, memory to receive it
 * \param ident_csr_len The identity CSR's memory region length
 *
 * \return 0 on success, -1 on failure.
 */
int identity_generate_ident_key_csr(uint8_t *ident_key, size_t ident_key_len, uint8_t *ident_csr, size_t ident_csr_len);

/**
 * Generate the device's identity key.
 *
 * \param ident_key_der The identity key, serialized as DER.
 * \param ident_key_max The size of the buffer to receive the identity key.
 * \param pident_key_len The actual length of the serialized DER key.
 */
int identity_generate_ident_key(uint8_t *ident_key_der, size_t ident_key_max, size_t *pident_key_len);

