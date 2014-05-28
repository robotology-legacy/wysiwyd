// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <opcManager_IDL.h>
#include <yarp/os/idl/WireTypes.h>



class opcManager_IDL_connect : public yarp::os::Portable {
public:
  std::string nameOPC;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(2)) return false;
    if (!writer.writeTag("connect",1,1)) return false;
    if (!writer.writeString(nameOPC)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

class opcManager_IDL_updateBeliefs : public yarp::os::Portable {
public:
  std::string nameOPC;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(2)) return false;
    if (!writer.writeTag("updateBeliefs",1,1)) return false;
    if (!writer.writeString(nameOPC)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

class opcManager_IDL_synchronise : public yarp::os::Portable {
public:
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("synchronise",1,1)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

class opcManager_IDL_executeActivity : public yarp::os::Portable {
public:
  std::string actionName;
  std::string Argument;
  std::string object;
  std::string agent;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(5)) return false;
    if (!writer.writeTag("executeActivity",1,1)) return false;
    if (!writer.writeString(actionName)) return false;
    if (!writer.writeString(Argument)) return false;
    if (!writer.writeString(object)) return false;
    if (!writer.writeString(agent)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

class opcManager_IDL_diffOPC : public yarp::os::Portable {
public:
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("diffOPC",1,1)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

class opcManager_IDL_quit : public yarp::os::Portable {
public:
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("quit",1,1)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

bool opcManager_IDL::connect(const std::string& nameOPC) {
  bool _return = false;
  opcManager_IDL_connect helper;
  helper.nameOPC = nameOPC;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool opcManager_IDL::connect(const std::string& nameOPC)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool opcManager_IDL::updateBeliefs(const std::string& nameOPC) {
  bool _return = false;
  opcManager_IDL_updateBeliefs helper;
  helper.nameOPC = nameOPC;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool opcManager_IDL::updateBeliefs(const std::string& nameOPC)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool opcManager_IDL::synchronise() {
  bool _return = false;
  opcManager_IDL_synchronise helper;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool opcManager_IDL::synchronise()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool opcManager_IDL::executeActivity(const std::string& actionName, const std::string& Argument, const std::string& object, const std::string& agent) {
  bool _return = false;
  opcManager_IDL_executeActivity helper;
  helper.actionName = actionName;
  helper.Argument = Argument;
  helper.object = object;
  helper.agent = agent;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool opcManager_IDL::executeActivity(const std::string& actionName, const std::string& Argument, const std::string& object, const std::string& agent)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool opcManager_IDL::diffOPC() {
  bool _return = false;
  opcManager_IDL_diffOPC helper;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool opcManager_IDL::diffOPC()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool opcManager_IDL::quit() {
  bool _return = false;
  opcManager_IDL_quit helper;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool opcManager_IDL::quit()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool opcManager_IDL::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "connect") {
      std::string nameOPC;
      if (!reader.readString(nameOPC)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = connect(nameOPC);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "updateBeliefs") {
      std::string nameOPC;
      if (!reader.readString(nameOPC)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = updateBeliefs(nameOPC);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "synchronise") {
      bool _return;
      _return = synchronise();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "executeActivity") {
      std::string actionName;
      std::string Argument;
      std::string object;
      std::string agent;
      if (!reader.readString(actionName)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(Argument)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(object)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(agent)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = executeActivity(actionName,Argument,object,agent);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "diffOPC") {
      bool _return;
      _return = diffOPC();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "quit") {
      bool _return;
      _return = quit();
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

std::vector<std::string> opcManager_IDL::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("connect");
    helpString.push_back("updateBeliefs");
    helpString.push_back("synchronise");
    helpString.push_back("executeActivity");
    helpString.push_back("diffOPC");
    helpString.push_back("quit");
    helpString.push_back("help");
  }
  else {
    if (functionName=="connect") {
      helpString.push_back("bool connect(const std::string& nameOPC) ");
      helpString.push_back("Connect the module to an OPC. ");
      helpString.push_back("@param nameOPC name of the OPC to connect (real or mental OPC). ");
      helpString.push_back("@return Bottle with state of the connection. ");
    }
    if (functionName=="updateBeliefs") {
      helpString.push_back("bool updateBeliefs(const std::string& nameOPC) ");
      helpString.push_back("Update the beliefs of the agents present in an OPC. ");
      helpString.push_back("@param nameOPC name of the OPC to update (real or mental OPC). ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="synchronise") {
      helpString.push_back("bool synchronise() ");
      helpString.push_back("Synchronise the content of the mentalOPC on the content of the realOPC ");
      helpString.push_back("@return Bottle with state of the synchronisation. ");
    }
    if (functionName=="executeActivity") {
      helpString.push_back("bool executeActivity(const std::string& actionName, const std::string& Argument, const std::string& object, const std::string& agent) ");
      helpString.push_back("Simulate the execution of an activity in the mental OPC. ");
      helpString.push_back("input format is the same as for abmReasoning: ");
      helpString.push_back("Bottle (\"executeActivity\"       actionName      argument        object      agent) ");
      helpString.push_back("@param name of the action to execute ");
      helpString.push_back("@param argument of the action to execute ");
      helpString.push_back("@param object of the action to execute ");
      helpString.push_back("@param agent of the action to execute ");
      helpString.push_back("@return Bottle with state of the execution. ");
    }
    if (functionName=="diffOPC") {
      helpString.push_back("bool diffOPC() ");
      helpString.push_back("Return the differences between the 2 OPCs ");
      helpString.push_back("@return Bottle of the differences ");
    }
    if (functionName=="quit") {
      helpString.push_back("bool quit() ");
      helpString.push_back("Quit the module. ");
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


