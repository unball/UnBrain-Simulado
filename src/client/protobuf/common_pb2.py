# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: common.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import builder as _builder
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x0c\x63ommon.proto\x12\x0c\x66ira_message\"K\n\x04\x42\x61ll\x12\t\n\x01x\x18\x01 \x01(\x01\x12\t\n\x01y\x18\x02 \x01(\x01\x12\t\n\x01z\x18\x03 \x01(\x01\x12\n\n\x02vx\x18\x04 \x01(\x01\x12\n\n\x02vy\x18\x05 \x01(\x01\x12\n\n\x02vz\x18\x06 \x01(\x01\"r\n\x05Robot\x12\x10\n\x08robot_id\x18\x01 \x01(\r\x12\t\n\x01x\x18\x02 \x01(\x01\x12\t\n\x01y\x18\x03 \x01(\x01\x12\x13\n\x0borientation\x18\x04 \x01(\x01\x12\n\n\x02vx\x18\x05 \x01(\x01\x12\n\n\x02vy\x18\x06 \x01(\x01\x12\x14\n\x0cvorientation\x18\x07 \x01(\x01\"N\n\x05\x46ield\x12\r\n\x05width\x18\x01 \x01(\x01\x12\x0e\n\x06length\x18\x02 \x01(\x01\x12\x12\n\ngoal_width\x18\x03 \x01(\x01\x12\x12\n\ngoal_depth\x18\x04 \x01(\x01\"\x7f\n\x05\x46rame\x12 \n\x04\x62\x61ll\x18\x01 \x01(\x0b\x32\x12.fira_message.Ball\x12*\n\rrobots_yellow\x18\x02 \x03(\x0b\x32\x13.fira_message.Robot\x12(\n\x0brobots_blue\x18\x03 \x03(\x0b\x32\x13.fira_message.Robotb\x06proto3')

_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, globals())
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'common_pb2', globals())
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _BALL._serialized_start=30
  _BALL._serialized_end=105
  _ROBOT._serialized_start=107
  _ROBOT._serialized_end=221
  _FIELD._serialized_start=223
  _FIELD._serialized_end=301
  _FRAME._serialized_start=303
  _FRAME._serialized_end=430
# @@protoc_insertion_point(module_scope)
