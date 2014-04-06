// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <cvz_mmcm_IDL.h>
#include <yarp/os/idl/WireTypes.h>



class cvz_mmcm_IDL_setLearningRate : public yarp::os::Portable {
public:
  double l;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(2)) return false;
    if (!writer.writeTag("setLearningRate",1,1)) return false;
    if (!writer.writeDouble(l)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    return true;
  }
};

class cvz_mmcm_IDL_getLearningRate : public yarp::os::Portable {
public:
  double _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("getLearningRate",1,1)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readDouble(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

class cvz_mmcm_IDL_setSigma : public yarp::os::Portable {
public:
  double s;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(2)) return false;
    if (!writer.writeTag("setSigma",1,1)) return false;
    if (!writer.writeDouble(s)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    return true;
  }
};

class cvz_mmcm_IDL_getSigma : public yarp::os::Portable {
public:
  double _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("getSigma",1,1)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readDouble(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

class cvz_mmcm_IDL_getActivity : public yarp::os::Portable {
public:
  int32_t x;
  int32_t y;
  int32_t z;
  double _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(4)) return false;
    if (!writer.writeTag("getActivity",1,1)) return false;
    if (!writer.writeI32(x)) return false;
    if (!writer.writeI32(y)) return false;
    if (!writer.writeI32(z)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readDouble(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

void cvz_mmcm_IDL::setLearningRate(const double l) {
  cvz_mmcm_IDL_setLearningRate helper;
  helper.l = l;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","void cvz_mmcm_IDL::setLearningRate(const double l)");
  }
  yarp().write(helper,helper);
}
double cvz_mmcm_IDL::getLearningRate() {
  double _return = (double)0;
  cvz_mmcm_IDL_getLearningRate helper;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","double cvz_mmcm_IDL::getLearningRate()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
void cvz_mmcm_IDL::setSigma(const double s) {
  cvz_mmcm_IDL_setSigma helper;
  helper.s = s;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","void cvz_mmcm_IDL::setSigma(const double s)");
  }
  yarp().write(helper,helper);
}
double cvz_mmcm_IDL::getSigma() {
  double _return = (double)0;
  cvz_mmcm_IDL_getSigma helper;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","double cvz_mmcm_IDL::getSigma()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
double cvz_mmcm_IDL::getActivity(const int32_t x, const int32_t y, const int32_t z) {
  double _return = (double)0;
  cvz_mmcm_IDL_getActivity helper;
  helper.x = x;
  helper.y = y;
  helper.z = z;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","double cvz_mmcm_IDL::getActivity(const int32_t x, const int32_t y, const int32_t z)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool cvz_mmcm_IDL::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "setLearningRate") {
      double l;
      if (!reader.readDouble(l)) {
        reader.fail();
        return false;
      }
      setLearningRate(l);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(0)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getLearningRate") {
      double _return;
      _return = getLearningRate();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeDouble(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setSigma") {
      double s;
      if (!reader.readDouble(s)) {
        reader.fail();
        return false;
      }
      setSigma(s);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(0)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getSigma") {
      double _return;
      _return = getSigma();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeDouble(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getActivity") {
      int32_t x;
      int32_t y;
      int32_t z;
      if (!reader.readI32(x)) {
        reader.fail();
        return false;
      }
      if (!reader.readI32(y)) {
        reader.fail();
        return false;
      }
      if (!reader.readI32(z)) {
        reader.fail();
        return false;
      }
      double _return;
      _return = getActivity(x,y,z);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeDouble(_return)) return false;
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

std::vector<std::string> cvz_mmcm_IDL::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("setLearningRate");
    helpString.push_back("getLearningRate");
    helpString.push_back("setSigma");
    helpString.push_back("getSigma");
    helpString.push_back("getActivity");
    helpString.push_back("help");
  }
  else {
    if (functionName=="setLearningRate") {
      helpString.push_back("void setLearningRate(const double l) ");
      helpString.push_back("Set the learning rate. ");
    }
    if (functionName=="getLearningRate") {
      helpString.push_back("double getLearningRate() ");
      helpString.push_back("Get the current learning rate. ");
    }
    if (functionName=="setSigma") {
      helpString.push_back("void setSigma(const double s) ");
      helpString.push_back("Set the neighborhood radius (sigma). ");
    }
    if (functionName=="getSigma") {
      helpString.push_back("double getSigma() ");
      helpString.push_back("Get the current neighborhood radius (sigma). ");
    }
    if (functionName=="getActivity") {
      helpString.push_back("double getActivity(const int32_t x, const int32_t y, const int32_t z) ");
      helpString.push_back("Get the activity of a neuron. ");
      helpString.push_back("@param x the horizontal coordinate of the neuron. ");
      helpString.push_back("@param y the vertical coordinate of the neuron. ");
      helpString.push_back("@param z the layer to which the neuron belongs. ");
      helpString.push_back("@return Current activity of the neuron. ");
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


