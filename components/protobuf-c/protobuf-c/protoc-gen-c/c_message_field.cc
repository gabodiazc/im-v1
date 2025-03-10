// Protocol Buffers - Google's data interchange format
// Copyright 2008 Google Inc.  All rights reserved.
// http://code.google.com/p/protobuf/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//     * Neither the name of Google Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Author: kenton@google.com (Kenton Varda)
//  Based on original Protocol Buffers design by
//  Sanjay Ghemawat, Jeff Dean, and others.

// Copyright (c) 2008-2025, Dave Benson and the protobuf-c authors.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Modified to implement C code by Dave Benson.

#include <google/protobuf/descriptor.h>
#include <google/protobuf/io/printer.h>
#include <google/protobuf/wire_format.h>

#include "c_helpers.h"
#include "c_message_field.h"

namespace protobuf_c {

MessageFieldGenerator::
MessageFieldGenerator(const google::protobuf::FieldDescriptor* descriptor)
  : FieldGenerator(descriptor) {
}

MessageFieldGenerator::~MessageFieldGenerator() {}

void MessageFieldGenerator::GenerateStructMembers(google::protobuf::io::Printer* printer) const
{
  std::map<std::string, std::string> vars;
  vars["name"] = FieldName(descriptor_);
  vars["type"] = FullNameToC(descriptor_->message_type()->full_name(), descriptor_->message_type()->file());
  vars["deprecated"] = FieldDeprecated(descriptor_);
  switch (descriptor_->label()) {
    case google::protobuf::FieldDescriptor::LABEL_REQUIRED:
    case google::protobuf::FieldDescriptor::LABEL_OPTIONAL:
      printer->Print(vars, "$type$ *$name$$deprecated$;\n");
      break;
    case google::protobuf::FieldDescriptor::LABEL_REPEATED:
      printer->Print(vars, "size_t n_$name$$deprecated$;\n");
      printer->Print(vars, "$type$ **$name$$deprecated$;\n");
      break;
  }
}
std::string MessageFieldGenerator::GetDefaultValue(void) const
{
  /* XXX: update when protobuf gets support
   *   for default-values of message fields.
   */
  return "NULL";
}
void MessageFieldGenerator::GenerateStaticInit(google::protobuf::io::Printer* printer) const
{
  switch (descriptor_->label()) {
    case google::protobuf::FieldDescriptor::LABEL_REQUIRED:
    case google::protobuf::FieldDescriptor::LABEL_OPTIONAL:
      printer->Print("NULL");
      break;
    case google::protobuf::FieldDescriptor::LABEL_REPEATED:
      printer->Print("0,NULL");
      break;
  }
}
void MessageFieldGenerator::GenerateDescriptorInitializer(google::protobuf::io::Printer* printer) const
{
  std::string addr = "&" + FullNameToLower(descriptor_->message_type()->full_name(), descriptor_->message_type()->file()) + "__descriptor";
  GenerateDescriptorInitializerGeneric(printer, false, "MESSAGE", addr);
}

}  // namespace protobuf_c
