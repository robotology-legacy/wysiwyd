// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <slidingController_IDLServer.h>
#include <yarp/os/idl/WireTypes.h>



class slidingController_IDLServer_stop : public yarp::os::Portable {
public:
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("stop",1,1)) return false;
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

class slidingController_IDLServer_calibrate : public yarp::os::Portable {
public:
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("calibrate",1,1)) return false;
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

class slidingController_IDLServer_impedance : public yarp::os::Portable {
public:
  std::string sw;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(2)) return false;
    if (!writer.writeTag("impedance",1,1)) return false;
    if (!writer.writeString(sw)) return false;
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

class slidingController_IDLServer_explore : public yarp::os::Portable {
public:
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("explore",1,1)) return false;
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

class slidingController_IDLServer_hand : public yarp::os::Portable {
public:
  std::string key;
  bool wait;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeTag("hand",1,1)) return false;
    if (!writer.writeString(key)) return false;
    if (!writer.writeBool(wait)) return false;
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

class slidingController_IDLServer_quit : public yarp::os::Portable {
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

bool slidingController_IDLServer::stop() {
  bool _return = false;
  slidingController_IDLServer_stop helper;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool slidingController_IDLServer::stop()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool slidingController_IDLServer::calibrate() {
  bool _return = false;
  slidingController_IDLServer_calibrate helper;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool slidingController_IDLServer::calibrate()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool slidingController_IDLServer::impedance(const std::string& sw) {
  bool _return = false;
  slidingController_IDLServer_impedance helper;
  helper.sw = sw;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool slidingController_IDLServer::impedance(const std::string& sw)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool slidingController_IDLServer::explore() {
  bool _return = false;
  slidingController_IDLServer_explore helper;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool slidingController_IDLServer::explore()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool slidingController_IDLServer::hand(const std::string& key, const bool wait) {
  bool _return = false;
  slidingController_IDLServer_hand helper;
  helper.key = key;
  helper.wait = wait;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool slidingController_IDLServer::hand(const std::string& key, const bool wait)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool slidingController_IDLServer::quit() {
  bool _return = false;
  slidingController_IDLServer_quit helper;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool slidingController_IDLServer::quit()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool slidingController_IDLServer::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "stop") {
      bool _return;
      _return = stop();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "calibrate") {
      bool _return;
      _return = calibrate();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "impedance") {
      std::string sw;
      if (!reader.readString(sw)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = impedance(sw);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "explore") {
      bool _return;
      _return = explore();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "hand") {
      std::string key;
      bool wait;
      if (!reader.readString(key)) {
        reader.fail();
        return false;
      }
      if (!reader.readBool(wait)) {
        wait = 0;
      }
      bool _return;
      _return = hand(key,wait);
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

std::vector<std::string> slidingController_IDLServer::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("stop");
    helpString.push_back("calibrate");
    helpString.push_back("impedance");
    helpString.push_back("explore");
    helpString.push_back("hand");
    helpString.push_back("quit");
    helpString.push_back("help");
  }
  else {
    if (functionName=="stop") {
      helpString.push_back("bool stop() ");
      helpString.push_back("Yield an immediate stop of any ongoing movements ");
      helpString.push_back("except exploration. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="calibrate") {
      helpString.push_back("bool calibrate() ");
      helpString.push_back("Yield a fingers recalibration. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="impedance") {
      helpString.push_back("bool impedance(const std::string& sw) ");
      helpString.push_back("Enable/disable impedance mode. ");
      helpString.push_back("@param sw is \"on\"|\"off\". ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="explore") {
      helpString.push_back("bool explore() ");
      helpString.push_back("Start off table exploration. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="hand") {
      helpString.push_back("bool hand(const std::string& key, const bool wait = 0) ");
      helpString.push_back("Execute an hand posture. ");
      helpString.push_back("@param key the tag of the hand posture. ");
      helpString.push_back("@param wait if true wait until posture is reached. ");
      helpString.push_back("@return true/false on success/failure. ");
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


