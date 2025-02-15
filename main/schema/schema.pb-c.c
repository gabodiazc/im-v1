/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: schema.proto */

/* Do not generate deprecated warnings for self */
#ifndef PROTOBUF_C__NO_DEPRECATED
#define PROTOBUF_C__NO_DEPRECATED
#endif

#include "schema.pb-c.h"
void   measure__init
                     (Measure         *message)
{
  static const Measure init_value = MEASURE__INIT;
  *message = init_value;
}
size_t measure__get_packed_size
                     (const Measure *message)
{
  assert(message->base.descriptor == &measure__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
size_t measure__pack
                     (const Measure *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &measure__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
size_t measure__pack_to_buffer
                     (const Measure *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &measure__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
Measure *
       measure__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (Measure *)
     protobuf_c_message_unpack (&measure__descriptor,
                                allocator, len, data);
}
void   measure__free_unpacked
                     (Measure *message,
                      ProtobufCAllocator *allocator)
{
  if(!message)
    return;
  assert(message->base.descriptor == &measure__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
static const ProtobufCFieldDescriptor measure__field_descriptors[16] =
{
  {
    "gyr_x_rads",
    1,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Measure, gyr_x_rads),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "gyr_y_rads",
    2,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Measure, gyr_y_rads),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "gyr_z_rads",
    3,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Measure, gyr_z_rads),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "acc_x_g",
    4,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Measure, acc_x_g),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "acc_y_g",
    5,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Measure, acc_y_g),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "acc_z_g",
    6,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Measure, acc_z_g),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "acc_x_ms2",
    7,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Measure, acc_x_ms2),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "acc_y_ms2",
    8,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Measure, acc_y_ms2),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "acc_z_ms2",
    9,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Measure, acc_z_ms2),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "mag_x_ut",
    10,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Measure, mag_x_ut),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "mag_y_ut",
    11,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Measure, mag_y_ut),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "mag_z_ut",
    12,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Measure, mag_z_ut),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "temp_c",
    13,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Measure, temp_c),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "hum_percent",
    14,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Measure, hum_percent),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "press_hpa",
    15,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Measure, press_hpa),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "gas_res_ohms",
    16,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(Measure, gas_res_ohms),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
static const unsigned measure__field_indices_by_name[] = {
  3,   /* field[3] = acc_x_g */
  6,   /* field[6] = acc_x_ms2 */
  4,   /* field[4] = acc_y_g */
  7,   /* field[7] = acc_y_ms2 */
  5,   /* field[5] = acc_z_g */
  8,   /* field[8] = acc_z_ms2 */
  15,   /* field[15] = gas_res_ohms */
  0,   /* field[0] = gyr_x_rads */
  1,   /* field[1] = gyr_y_rads */
  2,   /* field[2] = gyr_z_rads */
  13,   /* field[13] = hum_percent */
  9,   /* field[9] = mag_x_ut */
  10,   /* field[10] = mag_y_ut */
  11,   /* field[11] = mag_z_ut */
  14,   /* field[14] = press_hpa */
  12,   /* field[12] = temp_c */
};
static const ProtobufCIntRange measure__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 16 }
};
const ProtobufCMessageDescriptor measure__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "Measure",
  "Measure",
  "Measure",
  "",
  sizeof(Measure),
  16,
  measure__field_descriptors,
  measure__field_indices_by_name,
  1,  measure__number_ranges,
  (ProtobufCMessageInit) measure__init,
  NULL,NULL,NULL    /* reserved[123] */
};
