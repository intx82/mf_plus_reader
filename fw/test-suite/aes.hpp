#ifndef AES_H_
#define AES_H_

/**< Из: https://github.com/DavyLandman/AESLib */

class AES_T
	{
	public:

		struct roundkey_t
			{
			uint8_t ks[16];
			} ;


		struct cipher_state_t
			{
			uint8_t s[16];
			} ;

		/**
		* Состояние автомата
		*/
		struct aes128_ctx_t
			{
			roundkey_t key[10+1];
			} ;
		/**
		 * \brief initialize the keyschedule
		 *
		 * This function computes the keyschedule from a given key with a given length
		 * and stores it in the context variable
		 * \param key       pointer to the key material
		 * \param keysize_b length of the key in bits (valid are 128, 192 and 256)
		 * \param ctx       pointer to the context where the keyschedule should be stored
		 */
		void init(void* key, uint16_t keysize_b, aes128_ctx_t* ctx);
		uint8_t gf256mul(uint8_t a, uint8_t b, uint8_t reducer);
		static void rotword(void* a);
		void shiftcol(void* data, uint8_t shift);
		void enc_round(cipher_state_t* state, const roundkey_t* k);
		void enc_lastround(cipher_state_t* state,const roundkey_t* k);
		void encrypt_core(cipher_state_t* state, const aes128_ctx_t* ks, uint8_t rounds);
		void invshiftrow(void* data, uint8_t shift);
		void invshiftcol(void* data, uint8_t shift);
		void dec_round(cipher_state_t* state, const roundkey_t* k);
		void dec_firstround(cipher_state_t* state, const roundkey_t* k);
		void decrypt_core(cipher_state_t* state, const aes128_ctx_t* ks, uint8_t rounds);

		enum mode_t
			{
			ECB = 0,
			CBC
			};
		/**
		* Пустой конструктор
		*/
		AES_T()
			{

			}

		uint8_t iv_enc[16];
		uint8_t iv_dec[16];
		/**
		 * \brief initialize the keyschedule for 256 bit key
		 *
		 * This function computes the keyschedule from a given 256 bit key
		 * and stores it in the context variable
		 * \param key       pointer to the key material
		 * \param ctx       pointer to the context where the keyschedule should be stored
		 */
		void aes128_init(void* key);
		void aes128_dec(uint8_t* key, uint8_t* data, mode_t mode = ECB, uint8_t* iv = 0);

		void _xor(uint8_t* a, uint8_t* b, uint8_t a_len = 16, uint8_t b_len = 16);
		
		void aes128_enc(uint8_t* key,uint8_t* data,mode_t mode = ECB, uint8_t* iv = 0);
	};
#endif
