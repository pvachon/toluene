#include "identity.h"
#include "trusted_pk.h"

#include "esp_log.h"

#include <mbedtls/ecdsa.h>
#include <mbedtls/sha256.h>
#include <mbedtls/pk.h>
#include <mbedtls/asn1.h>

#include <string.h>

#define TAG                         "IDENT"

static
int _identity_check_signature(void const *message, size_t length, mbedtls_mpi const *r, mbedtls_mpi const *s, bool *pverify)
{
    int ret = -1;

    uint8_t hash[32];

    mbedtls_sha256_context ctx;
    mbedtls_pk_context pk_ctx;
    mbedtls_ecp_keypair *kp = NULL;

    mbedtls_sha256_init(&ctx);
    mbedtls_pk_init(&pk_ctx);

    if (NULL == pverify || NULL == message || 0 == length || NULL == r || NULL == s) {
        goto done;
    }

    *pverify = false;

    /* Load the trusted public key so we can verify the identity blob */
    if (mbedtls_pk_parse_public_key(&pk_ctx, pem_pub_key, sizeof(pem_pub_key))) {
        ESP_LOGE(TAG, "Failed to load the trusted public key, aborting.");
        goto done;
    }

    if (NULL == (kp = mbedtls_pk_ec(pk_ctx))) {
        ESP_LOGE(TAG, "Failed to get EC key pair, aborting.");
        goto done;
    }

    /* Hash the message */
    if (mbedtls_sha256_starts_ret(&ctx, 0)) {
        ESP_LOGE(TAG, "Failed to start SHA256, aborting.");
        goto done;
    }

    if (mbedtls_sha256_update_ret(&ctx, message, length)) {
        ESP_LOGE(TAG, "Failed to update with added data, aborting.");
        goto done;
    }

    if (mbedtls_sha256_finish_ret(&ctx, hash)) {
        ESP_LOGE(TAG, "Failed to finalize SHA256, aborting.");
        goto done;
    }

    /* Verify the signature */
    if (mbedtls_ecdsa_verify(&kp->grp, hash, 32, &kp->Q, r, s)) {
        ESP_LOGE(TAG, "Failed to verify signature, aborting.");
        goto done;
    }

    ESP_LOGI(TAG, "Verification complete.");

    *pverify = true;

    ret = 0;
done:
    mbedtls_pk_free(&pk_ctx);
    mbedtls_sha256_free(&ctx);
    return ret;
}

static
int _identity_extract_parameters(struct identity *ident, void *info_blob, size_t info_blob_len)
{
    int ret = -1;

    uint8_t *ib_p = info_blob,
            *ib_p_end = info_blob + info_blob_len;

    size_t essid_len = 0,
           wpa_psk_len = 0,
           target_host_len = 0;
    int target_host_port = -1,
        device_id = -1;

    if (NULL == info_blob || 0 == info_blob_len) {
        ESP_LOGE(TAG, "Invalid arguments, aborting.");
        goto done;
    }

    /* Extract the parameters */
    if (mbedtls_asn1_get_tag(&ib_p, ib_p_end, &essid_len, MBEDTLS_ASN1_UTF8_STRING)) {
        ESP_LOGE(TAG, "Failed to get ESSID tag, abortig.");
        goto done;
    }

    if (essid_len > IDENTITY_ESSID_LEN_MAX - 1) {
        ESP_LOGE(TAG, "ESSID tag is longer than supported, aborting.");
        goto done;
    }

    memcpy(ident->wifi_essid, ib_p, essid_len);

    ib_p += essid_len;

    if (mbedtls_asn1_get_tag(&ib_p, ib_p_end, &wpa_psk_len, MBEDTLS_ASN1_UTF8_STRING)) {
        ESP_LOGE(TAG, "Failed to get password tag, aborting");
        goto done;
    }

    if (wpa_psk_len > IDENTITY_PASSWORD_LEN_MAX - 1) {
        ESP_LOGE(TAG, "Password tag is longer than supported, aborting.");
        goto done;
    }

    memcpy(ident->wifi_password, ib_p, wpa_psk_len);

    ib_p += wpa_psk_len;

    if (mbedtls_asn1_get_tag(&ib_p, ib_p_end, &target_host_len, MBEDTLS_ASN1_UTF8_STRING)) {
        ESP_LOGE(TAG, "Failed to get target host, aborting.");
        goto done;
    }

    if (target_host_len > IDENTITY_TARGET_HOST_LEN_MAX - 1) {
        ESP_LOGE(TAG, "Target host is too long, aborting.");
        goto done;
    }

    memcpy(ident->target_host, ib_p, target_host_len);

    ib_p += target_host_len;

    if (mbedtls_asn1_get_int(&ib_p, ib_p_end, &target_host_port)) {
        ESP_LOGE(TAG, "Failed to get target host service port number, aborting.");
        goto done;
    }

    ident->target_port = target_host_port;

    if (mbedtls_asn1_get_int(&ib_p, ib_p_end, &device_id)) {
        ESP_LOGE(TAG, "Failed to get device ID, aborting.");
        goto done;
    }

    ident->device_id = device_id;

#ifdef CONFIG_DUMP_FLASH_CONFIG
    ESP_LOGI(TAG, "Target ESSID: %s", ident->wifi_essid);
    ESP_LOGI(TAG, "Target PSK: %s", ident->wifi_password);
    ESP_LOGI(TAG, "Connecting to: %s", ident->target_host);
    ESP_LOGI(TAG, "Target port: %u", ident->target_port);
    ESP_LOGI(TAG, "Device ID: %u", ident->device_id);
#endif

    ret = 0;
done:
    return ret;
}

int identity_read(struct identity *pident, void *bundle, size_t bundle_length)
{
    int ret = -1;

    bool sig_result = false;
    uint8_t *info_blob = NULL;
    uint8_t *bndl_p = bundle,
            *bndl_end = bundle + bundle_length;
    size_t info_blob_len = 0,
           sig_blob_len = 0,
           bndl_len = 0;
    mbedtls_mpi r,
                s;

    mbedtls_mpi_init(&r);
    mbedtls_mpi_init(&s);

    if (NULL == bundle || 0 == bundle_length) {
        ESP_LOGE(TAG, "Malformed device identity bundle provided, aborting.");
        goto done;
    }

    if (NULL == pident) {
        ESP_LOGE(TAG, "Malformed target configuration structure, aborting.");
        goto done;
    }

    memset(pident, 0, sizeof(struct identity));

    /* Grab (r, s) from the outer sequence */
    if (mbedtls_asn1_get_tag(&bndl_p, bndl_end, &bndl_len, MBEDTLS_ASN1_CONSTRUCTED | MBEDTLS_ASN1_SEQUENCE)) {
        ESP_LOGI(TAG, "Failed to get outer, aborting.");
        goto done;
    }

    /* Grab outer info sequence */
    int asnret = 0;
    if ((asnret = mbedtls_asn1_get_tag(&bndl_p, bndl_end, &info_blob_len, MBEDTLS_ASN1_SEQUENCE | MBEDTLS_ASN1_CONSTRUCTED))) {
        ESP_LOGI(TAG, "Failed to get info bundle tag, aborting. (got %d)", asnret);
        goto done;
    }
    info_blob = bndl_p;
    bndl_p += info_blob_len;

    /* Grab outer signature sequence, and r, s */
    if (mbedtls_asn1_get_tag(&bndl_p, bndl_end, &sig_blob_len, MBEDTLS_ASN1_SEQUENCE | MBEDTLS_ASN1_CONSTRUCTED)) {
        ESP_LOGI(TAG, "Failed to get signature tag, aborting.");
        goto done;
    }

    if (mbedtls_asn1_get_mpi(&bndl_p, bndl_end, &r)) {
        ESP_LOGI(TAG, "Failed to get signature r");
        goto done;
    }

    if (mbedtls_asn1_get_mpi(&bndl_p, bndl_end, &s)) {
        ESP_LOGI(TAG, "Failed to get signature s, aborting.");
        goto done;
    }

    /* Verify the signature on the blob */
    if (_identity_check_signature(info_blob - 2, info_blob_len + 2, &r, &s, &sig_result)) {
        ESP_LOGE(TAG, "Failed to verify signature (an internal error); aborting");
        goto done;
    }

    if (_identity_extract_parameters(pident, info_blob, info_blob_len)) {
        ESP_LOGE(TAG, "Failed to extract configuration parameters, aborting");
        goto done;
    }

    if (false == sig_result) {
        ESP_LOGE(TAG, "Fatal: incorrect signature on identity info bundle, aborting.");
        goto done;
    }

    /* TODO: verify the identity certificate chain, to make sure it is valid */

    ret = 0;
done:
    mbedtls_mpi_free(&r);
    mbedtls_mpi_free(&s);
    return ret;
}

