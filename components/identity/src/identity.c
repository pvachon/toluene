#include "identity.h"
#include "trusted_pk.h"

#include "DeviceIdentityBundle.h"
#include "DeviceIdentityInfo.h"
#include "ber_decoder.h"
#include "der_encoder.h"

#include "esp_log.h"

#include <mbedtls/ecdsa.h>
#include <mbedtls/sha256.h>
#include <mbedtls/pk.h>

#define TAG                         "IDENT"

static
int _encoder_get_length(const void *buf, size_t len, void *callback_key)
{
    uint32_t *lenp = callback_key;

    *lenp += len;

    return 0;
}

static
int _encoder_write_output(const void *buf, size_t len, void *callback_key)
{
    uint8_t **pcb = callback_key;

    // ESP_LOGI(TAG, "Len = %zu, cbk = %p *cpk = %p", len, pcb, *pcb);

    memcpy(*pcb, buf, len);
    *pcb += len;

    return 0;
}

/**
 * Decode the device identity info bundle
 */
static
int _identity_decode_asn1_bundle(void const *blob, size_t blob_len, DeviceIdentityBundle_t *bndl)
{
    int ret = -1;

    asn_dec_rval_t dec_ret;

    ESP_LOGI(TAG, "Decoding ASN.1 bundle encoding");

    if (NULL == bndl) {
        ESP_LOGE(TAG, "Programmer error: NULL reference to device identity info provided.");
        abort();
    }

    dec_ret = ber_decode(NULL, &asn_DEF_DeviceIdentityBundle, (void **)&bndl, blob, blob_len);
    if (dec_ret.code != RC_OK) {
        ESP_LOGE(TAG, "Bad DeviceIdentityInfo provided, aborting (reason: %d, consumed %zu).", dec_ret.code, dec_ret.consumed);
        goto done;
    }

    ret = 0;
done:
    if (0 != ret) {
        if (NULL !=  bndl) {
            ASN_STRUCT_FREE_CONTENTS_ONLY(asn_DEF_DeviceIdentityBundle, bndl);
            bndl = NULL;
        }
    }
    return ret;
}

static
int _identity_check_signature(void const *message, size_t length, uint8_t const *r_raw, size_t r_len, uint8_t const *s_raw, size_t s_len, bool *pverify)
{
    int ret = -1;

    uint8_t hash[32];

    mbedtls_mpi r,
                s;
    mbedtls_sha256_context ctx;
    mbedtls_pk_context pk_ctx;
    mbedtls_ecp_keypair *kp = NULL;

    mbedtls_mpi_init(&r);
    mbedtls_mpi_init(&s);
    mbedtls_sha256_init(&ctx);
    mbedtls_pk_init(&pk_ctx);

    if (NULL == pverify || NULL == message || 0 == length || NULL == r_raw || 0 == r_len || NULL == s_raw || 0 == s_len) {
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

    /* Read in R, S for the signature */
    ESP_LOGI(TAG, "R length = %zu", r_len);
    if (mbedtls_mpi_read_binary(&r, r_raw, r_len)) {
        ESP_LOGE(TAG, "Failed to load r for signature");
        goto done;
    }

    ESP_LOGI(TAG, "S length = %zu", s_len);
    if (mbedtls_mpi_read_binary(&s, s_raw, s_len)) {
        ESP_LOGE(TAG, "Failed to load s for signature");
        goto done;
    }

    ESP_LOGI(TAG, "Hash of blob:");
    ESP_LOG_BUFFER_HEXDUMP(TAG, hash, 32, ESP_LOG_INFO);

    /* Verify the signature */
    if (mbedtls_ecdsa_verify(&kp->grp, hash, 32, &kp->Q, &r, &s)) {
        ESP_LOGE(TAG, "Failed to verify signature, aborting.");
        goto done;
    }

    ESP_LOGI(TAG, "Verification complete.");

    *pverify = true;

    ret = 0;
done:
    mbedtls_pk_free(&pk_ctx);
    mbedtls_sha256_free(&ctx);
    mbedtls_mpi_free(&s);
    mbedtls_mpi_free(&r);
    return ret;
}

static
int _identity_verify_info_signature(DeviceIdentityBundle_t *bndl, bool *psig_result)
{
    int ret = -1;

    uint32_t size = 0;
    void *ber_bndl = NULL,
         *ber_bndl_p = NULL;

    asn_enc_rval_t rval;

    if (NULL == psig_result || NULL == bndl) {
        ESP_LOGE(TAG, "Invalid arguments, aborting.");
        goto done;
    }

    *psig_result = false;

    /* Re-create the bundle */
    rval = der_encode(&asn_DEF_DeviceIdentityInfo, (void *)bndl, _encoder_get_length, (void *)&size);
    if (rval.encoded == -1) {
        goto done;
    }

    if (0 == size) {
        ESP_LOGE(TAG, "Failure: could not get length of encoded bundle, aborting.");
        goto done;
    }

    if (NULL == (ber_bndl = malloc(size))) {
        ESP_LOGE(TAG, "failure: could not allocate memory for new bundle, aborting.");
        goto done;
    }

    ber_bndl_p = ber_bndl;
    rval = der_encode(&asn_DEF_DeviceIdentityInfo, (void *)bndl, _encoder_write_output, (void *)&ber_bndl_p);
    if (rval.encoded == -1) {
        ESP_LOGE(TAG, "Failure: could not re-encode identity info, aborting.");
        goto done;
    }

    ESP_LOGI(TAG, "Re-encoded DER data");
    ESP_LOG_BUFFER_HEXDUMP(TAG, ber_bndl, size, ESP_LOG_INFO);

    /* Extract the signature and verify it */
    if (_identity_check_signature(ber_bndl, size,
                bndl->identityInfoSignature.r.buf, bndl->identityInfoSignature.r.size,
                bndl->identityInfoSignature.s.buf, bndl->identityInfoSignature.s.size,
                psig_result))
    {
        ESP_LOGE(TAG, "Failed to verify signature (an internal error); aborting");
        goto done;
    }

    ret = 0;
done:
    if (NULL != bndl) {
        free(bndl);
        bndl = NULL;
    }
    return ret;
}

int identity_read(struct identity *pident, void const *bundle, size_t bundle_length)
{
    int ret = -1;

    DeviceIdentityBundle_t bndl;
    DeviceIdentityInfo_t *info;
    bool sig_result = false;

    if (NULL == bundle || 0 == bundle_length) {
        ESP_LOGE(TAG, "Malformed device identity bundle provided, aborting.");
        goto done;
    }

    if (NULL == pident) {
        ESP_LOGE(TAG, "Malformed target configuration structure, aborting.");
        goto done;
    }

    memset(&bndl, 0, sizeof(bndl));

    ESP_LOGI(TAG, "Getting ready to decode device identity bundle (length=%zu)", bundle_length);
    ESP_LOG_BUFFER_HEXDUMP(TAG, bundle, bundle_length, ESP_LOG_INFO);

    if (_identity_decode_asn1_bundle(bundle, bundle_length, &bndl)) {
        ESP_LOGE(TAG, "Device identity bundle is malformed, aborting.");
        goto done;
    }

    info = &bndl.identityInfo;

    ESP_LOGI(TAG, "Verifying blob signature");

    /* Verify the signature on the blob */
    if (_identity_verify_info_signature(&bndl, &sig_result)) {
        ESP_LOGE(TAG, "Fatal: error checking signature for bundle, aborting.");
        goto done;
    }

    if (false == sig_result) {
        ESP_LOGE(TAG, "Fatal: incorrect signature on identity info bundle, aborting.");
        goto done;
    }

    /* TODO: verify the identity certificate chain, to make sure it is valid */


    if (info->wifiESSID.size > 63 || info->wifiPassword.size > 63 || info->targetHost.size > 127) {
        ESP_LOGE(TAG, "Fatal: ESSID, Password or Target Host information is too long, aborting.");
        goto done;
    }

    memset(pident, 0, sizeof(struct identity));

    /* Populate the identity structure */
    pident->target_port = info->targetPort;
    pident->device_id = info->deviceId;

    memcpy(pident->wifi_essid, info->wifiESSID.buf, info->wifiESSID.size);
    memcpy(pident->wifi_password, info->wifiPassword.buf, info->wifiPassword.size);
    memcpy(pident->target_host, info->targetHost.buf, info->targetHost.size);

    ret = 0;
done:
    ASN_STRUCT_FREE_CONTENTS_ONLY(asn_DEF_DeviceIdentityBundle, &bndl);

    return ret;
}

