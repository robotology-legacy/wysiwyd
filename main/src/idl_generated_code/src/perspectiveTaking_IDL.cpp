// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <perspectiveTaking_IDL.h>
#include <yarp/os/idl/WireTypes.h>



class perspectiveTaking_IDL_setUpdateTimer : public yarp::os::Portable {
public:
  int32_t interval;
  bool _return;
  void init(const int32_t interval);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class perspectiveTaking_IDL_setDecimationOdometry : public yarp::os::Portable {
public:
  int32_t decimation;
  bool _return;
  void init(const int32_t decimation);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class perspectiveTaking_IDL_setDecimationStatistics : public yarp::os::Portable {
public:
  int32_t decimation;
  bool _return;
  void init(const int32_t decimation);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class perspectiveTaking_IDL_processStats : public yarp::os::Portable {
public:
  bool enable;
  bool _return;
  void init(const bool enable);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool perspectiveTaking_IDL_setUpdateTimer::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("setUpdateTimer",1,1)) return false;
  if (!writer.writeI32(interval)) return false;
  return true;
}

bool perspectiveTaking_IDL_setUpdateTimer::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void perspectiveTaking_IDL_setUpdateTimer::init(const int32_t interval) {
  _return = false;
  this->interval = interval;
}

bool perspectiveTaking_IDL_setDecimationOdometry::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("setDecimationOdometry",1,1)) return false;
  if (!writer.writeI32(decimation)) return false;
  return true;
}

bool perspectiveTaking_IDL_setDecimationOdometry::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void perspectiveTaking_IDL_setDecimationOdometry::init(const int32_t decimation) {
  _return = false;
  this->decimation = decimation;
}

bool perspectiveTaking_IDL_setDecimationStatistics::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("setDecimationStatistics",1,1)) return false;
  if (!writer.writeI32(decimation)) return false;
  return true;
}

bool perspectiveTaking_IDL_setDecimationStatistics::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void perspectiveTaking_IDL_setDecimationStatistics::init(const int32_t decimation) {
  _return = false;
  this->decimation = decimation;
}

bool perspectiveTaking_IDL_processStats::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("processStats",1,1)) return false;
  if (!writer.writeBool(enable)) return false;
  return true;
}

bool perspectiveTaking_IDL_processStats::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void perspectiveTaking_IDL_processStats::init(const bool enable) {
  _return = false;
  this->enable = enable;
}

perspectiveTaking_IDL::perspectiveTaking_IDL() {
  yarp().setOwner(*this);
}
bool perspectiveTaking_IDL::setUpdateTimer(const int32_t interval) {
  bool _return = false;
  perspectiveTaking_IDL_setUpdateTimer helper;
  helper.init(interval);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool perspectiveTaking_IDL::setUpdateTimer(const int32_t interval)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool perspectiveTaking_IDL::setDecimationOdometry(const int32_t decimation) {
  bool _return = false;
  perspectiveTaking_IDL_setDecimationOdometry helper;
  helper.init(decimation);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool perspectiveTaking_IDL::setDecimationOdometry(const int32_t decimation)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool perspectiveTaking_IDL::setDecimationStatistics(const int32_t decimation) {
  bool _return = false;
  perspectiveTaking_IDL_setDecimationStatistics helper;
  helper.init(decimation);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool perspectiveTaking_IDL::setDecimationStatistics(const int32_t decimation)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool perspectiveTaking_IDL::processStats(const bool enable) {
  bool _return = false;
  perspectiveTaking_IDL_processStats helper;
  helper.init(enable);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool perspectiveTaking_IDL::processStats(const bool enable)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool perspectiveTaking_IDL::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "setUpdateTimer") {
      int32_t interval;
      if (!reader.readI32(interval)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = setUpdateTimer(interval);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setDecimationOdometry") {
      int32_t decimation;
      if (!reader.readI32(decimation)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = setDecimationOdometry(decimation);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setDecimationStatistics") {
      int32_t decimation;
      if (!reader.readI32(decimation)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = setDecimationStatistics(decimation);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "processStats") {
      bool enable;
      if (!reader.readBool(enable)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = processStats(enable);
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

std::vector<std::string> perspectiveTaking_IDL::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("setUpdateTimer");
    helpString.push_back("setDecimationOdometry");
    helpString.push_back("setDecimationStatistics");
    helpString.push_back("processStats");
    helpString.push_back("help");
  }
  else {
    if (functionName=="setUpdateTimer") {
      helpString.push_back("bool setUpdateTimer(const int32_t interval) ");
      helpString.push_back("Set the interval how often the ");
      helpString.push_back("third person perspective ");
      helpString.push_back("camera view should be updated ");
      helpString.push_back("@param interval is the interval in ms ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="setDecimationOdometry") {
      helpString.push_back("bool setDecimationOdometry(const int32_t decimation) ");
      helpString.push_back("Set the decimation (quality) of the most ");
      helpString.push_back("recent camera frame. The lower, the better ");
      helpString.push_back("@param decimation is a power of 2 (1, 2, 4, 8, ...) ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="setDecimationStatistics") {
      helpString.push_back("bool setDecimationStatistics(const int32_t decimation) ");
      helpString.push_back("Set the decimation (quality) of for the ");
      helpString.push_back("frames in the memory. The lower, the better ");
      helpString.push_back("@param decimation is a power of 2 (1, 2, 4, 8, ...) ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="processStats") {
      helpString.push_back("bool processStats(const bool enable) ");
      helpString.push_back("Decide whether or not new frames should ");
      helpString.push_back("be added to the memory or not ");
      helpString.push_back("@param enable sets whether the current frame ");
      helpString.push_back("is added to the memory or not ");
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


