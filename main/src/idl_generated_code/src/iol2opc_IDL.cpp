// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <iol2opc_IDL.h>
#include <yarp/os/idl/WireTypes.h>



class iol2opc_IDL_train_object : public yarp::os::Portable {
public:
  std::string name;
  bool _return;
  void init(const std::string& name);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class iol2opc_IDL_remove_object : public yarp::os::Portable {
public:
  std::string name;
  bool _return;
  void init(const std::string& name);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class iol2opc_IDL_remove_all : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class iol2opc_IDL_change_name : public yarp::os::Portable {
public:
  std::string old_name;
  std::string new_name;
  bool _return;
  void init(const std::string& old_name, const std::string& new_name);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool iol2opc_IDL_train_object::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("train_object",1,2)) return false;
  if (!writer.writeString(name)) return false;
  return true;
}

bool iol2opc_IDL_train_object::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void iol2opc_IDL_train_object::init(const std::string& name) {
  _return = false;
  this->name = name;
}

bool iol2opc_IDL_remove_object::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("remove_object",1,2)) return false;
  if (!writer.writeString(name)) return false;
  return true;
}

bool iol2opc_IDL_remove_object::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void iol2opc_IDL_remove_object::init(const std::string& name) {
  _return = false;
  this->name = name;
}

bool iol2opc_IDL_remove_all::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("remove_all",1,2)) return false;
  return true;
}

bool iol2opc_IDL_remove_all::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void iol2opc_IDL_remove_all::init() {
  _return = false;
}

bool iol2opc_IDL_change_name::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  if (!writer.writeTag("change_name",1,2)) return false;
  if (!writer.writeString(old_name)) return false;
  if (!writer.writeString(new_name)) return false;
  return true;
}

bool iol2opc_IDL_change_name::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void iol2opc_IDL_change_name::init(const std::string& old_name, const std::string& new_name) {
  _return = false;
  this->old_name = old_name;
  this->new_name = new_name;
}

iol2opc_IDL::iol2opc_IDL() {
  yarp().setOwner(*this);
}
bool iol2opc_IDL::train_object(const std::string& name) {
  bool _return = false;
  iol2opc_IDL_train_object helper;
  helper.init(name);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool iol2opc_IDL::train_object(const std::string& name)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool iol2opc_IDL::remove_object(const std::string& name) {
  bool _return = false;
  iol2opc_IDL_remove_object helper;
  helper.init(name);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool iol2opc_IDL::remove_object(const std::string& name)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool iol2opc_IDL::remove_all() {
  bool _return = false;
  iol2opc_IDL_remove_all helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool iol2opc_IDL::remove_all()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool iol2opc_IDL::change_name(const std::string& old_name, const std::string& new_name) {
  bool _return = false;
  iol2opc_IDL_change_name helper;
  helper.init(old_name,new_name);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool iol2opc_IDL::change_name(const std::string& old_name, const std::string& new_name)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool iol2opc_IDL::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "train_object") {
      std::string name;
      if (!reader.readString(name)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = train_object(name);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "remove_object") {
      std::string name;
      if (!reader.readString(name)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = remove_object(name);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "remove_all") {
      bool _return;
      _return = remove_all();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "change_name") {
      std::string old_name;
      std::string new_name;
      if (!reader.readString(old_name)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(new_name)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = change_name(old_name,new_name);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "help") {
      std::string functionName;
      if (!reader.readString(functionName)) {
        functionName = "--all";
      }
      std::vector<std::string> _return=help(functionName);
      yarp::os::idl::WireWriter writer(reader);
        if (!writer.isNull()) {
          if (!writer.writeListHeader(2)) return false;
          if (!writer.writeTag("many",1, 0)) return false;
          if (!writer.writeListBegin(BOTTLE_TAG_INT, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<std::string> ::iterator _iterHelp;
          for (_iterHelp = _return.begin(); _iterHelp != _return.end(); ++_iterHelp)
          {
            if (!writer.writeString(*_iterHelp)) return false;
           }
          if (!writer.writeListEnd()) return false;
        }
      reader.accept();
      return true;
    }
    if (reader.noMore()) { reader.fail(); return false; }
    yarp::os::ConstString next_tag = reader.readTag();
    if (next_tag=="") break;
    tag = tag + "_" + next_tag;
  }
  return false;
}

std::vector<std::string> iol2opc_IDL::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("train_object");
    helpString.push_back("remove_object");
    helpString.push_back("remove_all");
    helpString.push_back("change_name");
    helpString.push_back("help");
  }
  else {
    if (functionName=="train_object") {
      helpString.push_back("bool train_object(const std::string& name) ");
      helpString.push_back("Add a new object to the object-recognition database ");
      helpString.push_back("based on the selected blob. If the object is already ");
      helpString.push_back("existing, its recognition is improved. ");
      helpString.push_back("@param name is the object name ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="remove_object") {
      helpString.push_back("bool remove_object(const std::string& name) ");
      helpString.push_back("Remove object with a given name from the object-recognition ");
      helpString.push_back("database. ");
      helpString.push_back("@param name is the object name ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="remove_all") {
      helpString.push_back("bool remove_all() ");
      helpString.push_back("Remove all objects from the object-recognition ");
      helpString.push_back("database. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="change_name") {
      helpString.push_back("bool change_name(const std::string& old_name, const std::string& new_name) ");
      helpString.push_back("Change the name of an object ");
      helpString.push_back("@param old_name is the object which name is to be changed ");
      helpString.push_back("@param new_name is the new object name ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="help") {
      helpString.push_back("std::vector<std::string> help(const std::string& functionName=\"--all\")");
      helpString.push_back("Return list of available commands, or help message for a specific function");
      helpString.push_back("@param functionName name of command for which to get a detailed description. If none or '--all' is provided, print list of available commands");
      helpString.push_back("@return list of strings (one string per line)");
    }
  }
  if ( helpString.empty()) helpString.push_back("Command not found");
  return helpString;
}


