/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: sec0.proto */

#ifndef PROTOBUF_C_sec0_2eproto__INCLUDED
#define PROTOBUF_C_sec0_2eproto__INCLUDED

#include <protobuf-c/protobuf-c.h>

PROTOBUF_C__BEGIN_DECLS

#if PROTOBUF_C_VERSION_NUMBER < 1003000
# error This file was generated by a newer version of protoc-c which is incompatible with your libprotobuf-c headers. Please update your headers.
#elif 1003003 < PROTOBUF_C_MIN_COMPILER_VERSION
# error This file was generated by an older version of protoc-c which is incompatible with your libprotobuf-c headers. Please regenerate this file with a newer version of protoc-c.
#endif

#include "constants.pb-c.h"

typedef struct _S0SessionCmd S0SessionCmd;
typedef struct _S0SessionResp S0SessionResp;
typedef struct _Sec0Payload Sec0Payload;


/* --- enums --- */

/*
 * A message must be of type Cmd or Resp 
 */
typedef enum _Sec0MsgType {
  SEC0_MSG_TYPE__S0_Session_Command = 0,
  SEC0_MSG_TYPE__S0_Session_Response = 1
    PROTOBUF_C__FORCE_ENUM_TO_BE_INT_SIZE(SEC0_MSG_TYPE)
} Sec0MsgType;

/* --- messages --- */

/*
 * Data structure of Session command/request packet 
 */
struct  _S0SessionCmd
{
  ProtobufCMessage base;
};
#define S0_SESSION_CMD__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&s0_session_cmd__descriptor) \
     }


/*
 * Data structure of Session response packet 
 */
struct  _S0SessionResp
{
  ProtobufCMessage base;
  Status status;
};
#define S0_SESSION_RESP__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&s0_session_resp__descriptor) \
    , STATUS__Success }


typedef enum {
  SEC0_PAYLOAD__PAYLOAD__NOT_SET = 0,
  SEC0_PAYLOAD__PAYLOAD_SC = 20,
  SEC0_PAYLOAD__PAYLOAD_SR = 21
    PROTOBUF_C__FORCE_ENUM_TO_BE_INT_SIZE(SEC0_PAYLOAD__PAYLOAD)
} Sec0Payload__PayloadCase;

/*
 * Payload structure of session data 
 */
struct  _Sec0Payload
{
  ProtobufCMessage base;
  /*
   *!< Type of message 
   */
  Sec0MsgType msg;
  Sec0Payload__PayloadCase payload_case;
  union {
    /*
     *!< Payload data interpreted as Cmd 
     */
    S0SessionCmd *sc;
    /*
     *!< Payload data interpreted as Resp 
     */
    S0SessionResp *sr;
  };
};
#define SEC0_PAYLOAD__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&sec0_payload__descriptor) \
    , SEC0_MSG_TYPE__S0_Session_Command, SEC0_PAYLOAD__PAYLOAD__NOT_SET, {0} }


/* S0SessionCmd methods */
void   s0_session_cmd__init
                     (S0SessionCmd         *message);
size_t s0_session_cmd__get_packed_size
                     (const S0SessionCmd   *message);
size_t s0_session_cmd__pack
                     (const S0SessionCmd   *message,
                      uint8_t             *out);
size_t s0_session_cmd__pack_to_buffer
                     (const S0SessionCmd   *message,
                      ProtobufCBuffer     *buffer);
S0SessionCmd *
       s0_session_cmd__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   s0_session_cmd__free_unpacked
                     (S0SessionCmd *message,
                      ProtobufCAllocator *allocator);
/* S0SessionResp methods */
void   s0_session_resp__init
                     (S0SessionResp         *message);
size_t s0_session_resp__get_packed_size
                     (const S0SessionResp   *message);
size_t s0_session_resp__pack
                     (const S0SessionResp   *message,
                      uint8_t             *out);
size_t s0_session_resp__pack_to_buffer
                     (const S0SessionResp   *message,
                      ProtobufCBuffer     *buffer);
S0SessionResp *
       s0_session_resp__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   s0_session_resp__free_unpacked
                     (S0SessionResp *message,
                      ProtobufCAllocator *allocator);
/* Sec0Payload methods */
void   sec0_payload__init
                     (Sec0Payload         *message);
size_t sec0_payload__get_packed_size
                     (const Sec0Payload   *message);
size_t sec0_payload__pack
                     (const Sec0Payload   *message,
                      uint8_t             *out);
size_t sec0_payload__pack_to_buffer
                     (const Sec0Payload   *message,
                      ProtobufCBuffer     *buffer);
Sec0Payload *
       sec0_payload__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   sec0_payload__free_unpacked
                     (Sec0Payload *message,
                      ProtobufCAllocator *allocator);
/* --- per-message closures --- */

typedef void (*S0SessionCmd_Closure)
                 (const S0SessionCmd *message,
                  void *closure_data);
typedef void (*S0SessionResp_Closure)
                 (const S0SessionResp *message,
                  void *closure_data);
typedef void (*Sec0Payload_Closure)
                 (const Sec0Payload *message,
                  void *closure_data);

/* --- services --- */


/* --- descriptors --- */

extern const ProtobufCEnumDescriptor    sec0_msg_type__descriptor;
extern const ProtobufCMessageDescriptor s0_session_cmd__descriptor;
extern const ProtobufCMessageDescriptor s0_session_resp__descriptor;
extern const ProtobufCMessageDescriptor sec0_payload__descriptor;

PROTOBUF_C__END_DECLS


#endif  /* PROTOBUF_C_sec0_2eproto__INCLUDED */
