//=================================================================================================
// Copyright (c) 2011, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <introspection/field.h>
#include <introspection/type.h>
#include <introspection/introspection.h>
#include <introspection/message_expansion.h>

#include <introspection/dlfcn.h>
#include <iostream>

#define BOOST_FILESYSTEM_VERSION 3
#include <boost/filesystem.hpp>

namespace cpp_introspection {

   G_Vars * gvars;

  PackagePtr package(const std::string& pkg)
  {
    if (!gvars->g_packages.count(pkg)) return PackagePtr();
    return gvars->g_packages[pkg].lock();
  }

  const V_Package &packages() {
    return gvars->g_repository;
  }

  MessagePtr messageByDataType(const std::string& data_type, const std::string& package)
  {
    if (!package.empty()) return messageByDataType(package + "/" + data_type);
    if (data_type == "Header") return gvars->g_messages_by_name[ros::message_traits::datatype<std_msgs::Header>()].lock();
    if (!gvars->g_messages_by_name.count(data_type)) {
    	return MessagePtr();
    }
    MessagePtr message = gvars->g_messages_by_name[data_type].lock();
    return message;
  }

  MessagePtr messageByMD5Sum(const std::string& md5sum)
  {
    if (!gvars->g_messages_by_md5sum.count(md5sum)) return MessagePtr();
    return gvars->g_messages_by_md5sum[md5sum].lock();
  }

  MessagePtr messageByTypeId(const std::type_info &type_info) {
    if (!gvars->g_messages_by_typeid.count(&type_info)) return MessagePtr();
    return gvars->g_messages_by_typeid[&type_info].lock();
  }

  PackagePtr Package::add(const PackagePtr& package)
  {
    if (gvars->g_packages.count(package->getName())) return gvars->g_packages[package->getName()].lock();
    gvars->g_repository.push_back(package);
    gvars->g_packages[package->getName()] = package;
    return package;
  }

  std::vector<std::string> Package::getMessages() const
  {
    std::vector<std::string> messages;
    for(std::vector<MessagePtr>::const_iterator it = messages_.begin(); it != messages_.end(); ++it)
      messages.push_back((*it)->getName());

    return messages;
  }

  V_Message Package::getMessageObjects() const
  {
	return messages_;
  }

  MessagePtr Package::message(const std::string& message) const
  {
    return messageByDataType(std::string(getName()) + "/" + message);
  }

  MessagePtr Package::add(const MessagePtr & message) {

	  if (gvars_->g_messages_by_name.count(message->getDataType()))
		  return gvars_->g_messages_by_name[message->getDataType()].lock();
	  messages_.push_back(message);
	  gvars_->g_messages_by_name[message->getDataType()] = message;
	  gvars_->g_messages_by_md5sum[message->getMD5Sum()] = message;
	  gvars_->g_messages_by_typeid[&(message->getTypeId())] = message;
	  return message;

  }

  MessagePtr expand(const MessagePtr& message, const std::string &separator, const std::string &prefix)
  {
    return MessagePtr(new ExpandedMessage(message, separator, prefix));
  }

  void ExpandedMessage::expand(const MessagePtr &message, const std::string& prefix) {
    for(Message::const_iterator i = message->begin(); i != message->end(); i++) {
      FieldPtr field = *i;

      for(std::size_t j = 0; j < field->size(); j++) {
        std::string field_name = (!prefix.empty() ? prefix + separator_ : "") + field->getName();
        if (field->isContainer()) field_name += boost::lexical_cast<std::string>(j);

        if (field->isMessage()) {
          MessagePtr expanded(field->expand(j));
          if (!expanded) {
            ROS_WARN("Expansion of %s failed: Could not expand field %s with unknown type %s", getDataType(), field->getName(), field->getDataType());
            continue;
          }
          expand(expanded, field_name);
        } else {
          FieldPtr expanded(new ExpandedField(*field, field_name, j));
          fields_.push_back(expanded);
          fields_by_name_[field_name] = expanded;
          field_names_.push_back(expanded->getName());
        }
      }
    }
  }

  V_string Message::getFields(bool expand, const std::string& separator, const std::string& prefix) const
  {
    V_string fields;
    return getFields(fields, expand, separator, prefix);
  }

  V_string& Message::getFields(V_string& fields, bool expand, const std::string& separator, const std::string& prefix) const
  {
    for(const_iterator it = begin(); it != end(); ++it) {
      FieldPtr field = *it;
      std::string base(prefix + field->getName());

      for (std::size_t index = 0; index < field->size(); ++index) {
        std::string name(base);
        if (field->isArray()) name = name + boost::lexical_cast<std::string>(index);

        if (expand && field->isMessage()) {
          MessagePtr expanded = field->expand(index);
          if (!expanded) {
            ROS_WARN("Expansion of %s failed: Could not expand field %s with unknown type %s", getDataType(), field->getName(), field->getDataType());
            continue;
          }
          expanded->getFields(fields, expand, separator, name + separator);

        } else {
          fields.push_back(name);
        }
      }
    }

    return fields;
  }

  V_string Message::getTypes(bool expand) const
  {
    V_string types;
    return getTypes(types, expand);
  }

  V_string& Message::getTypes(V_string& types, bool expand) const
  {
    for(const_iterator it = begin(); it != end(); ++it) {
      FieldPtr field = *it;

      for (std::size_t index = 0; index < field->size(); ++index) {
        if (expand && field->isMessage()) {
          MessagePtr expanded = field->expand(index);
          if (!expanded) {
            ROS_WARN("Expansion of %s failed: Could not expand field %s with unknown type %s", getDataType(), field->getName(), field->getDataType());
            continue;
          }
          expanded->getTypes(types, expand);

        } else {
          types.push_back(field->isArray() ? field->getValueType() : field->getDataType());
        }
      }
    }

    return types;
  }

  std::vector<boost::any> Message::getValues(bool expand) const
  {
    std::vector<boost::any> values;
    return getValues(values, expand);
  }

  std::vector<boost::any>& Message::getValues(std::vector<boost::any>& values, bool expand) const
  {
    if (!hasInstance()) return values;

    for(const_iterator it = begin(); it != end(); ++it) {
      FieldPtr field = *it;

      for (std::size_t index = 0; index < field->size(); ++index) {
        if (expand && field->isMessage()) {
          MessagePtr expanded = field->expand(index);
          if (!expanded) {
            ROS_WARN("Expansion of %s failed: Could not expand field %s with unknown type %s", getDataType(), field->getName(), field->getDataType());
            continue;
          }
          expanded->getValues(values, expand);

        } else {
          values.push_back(field->get(index));
        }
      }
    }

    return values;
  }

  MessagePtr Field::expand(std::size_t i) const {
    if (!isMessage()) return MessagePtr();
    return messageByTypeId(this->getTypeId());
  }

  PackagePtr loadPackage(const std::string &package_name)
  {
    PackagePtr p = package(package_name);
    if (p) return p;
    return load("introspection_" + package_name + ".dll");
  }

  using namespace boost::filesystem;
  PackagePtr load(const std::string& package_or_library_or_path)
  {
    path path(package_or_library_or_path);
    if (is_directory(path)) {
//      ROS_DEBUG_STREAM_NAMED(ROS_PACKAGE_NAME, "Searching directory " << path << "...");
      for(directory_iterator entry(path); entry != directory_iterator(); ++entry) {
//        ROS_DEBUG_STREAM_NAMED(ROS_PACKAGE_NAME, "  " << *entry << "...");
        if (is_regular_file(entry->path())) load(entry->path().string());
      }

      return PackagePtr();
    }

    if (path.extension() != ".dll") {
      loadPackage(package_or_library_or_path);
      return PackagePtr();
    }

    if (std::find(gvars->g_loaded_libraries.begin(), gvars->g_loaded_libraries.end(), path.filename()) != gvars->g_loaded_libraries.end()) {
      return PackagePtr();
    }

    void *library = dlopen(path.string().c_str(), RTLD_NOW | RTLD_GLOBAL);
    const char *error = dlerror();
    if (error || !library) {
      ROS_ERROR("%s", error);
      return PackagePtr();
    }

    typedef PackagePtr (*LoadFunction)(struct G_Vars *);
    LoadFunction load_fcn = (LoadFunction) dlsym(library, "cpp_introspection_load_package");
    error = dlerror();
    if (error || !load_fcn) {
      dlclose(library);
      return PackagePtr();
    }
    PackagePtr package = (*load_fcn)(gvars);

    gvars->g_loaded_libraries.push_back(path.filename().string());

    // Copy the messages inside of the package instances into the global array
    const V_Message loaded_messages = package->getMessageObjects();
    for (V_Message::const_iterator it=loaded_messages.begin(); it!=loaded_messages.end();++it)
    {
    	gvars->g_messages_by_name[(*it)->getDataType()] = *it;
    }

    return package;
  }

} // namespace cpp_introspection
