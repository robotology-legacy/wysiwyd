#include <yarp/os/Bottle.h>
#include <wrdac/tags.h>
#include <string>
#include <vector>

namespace wysiwyd{
    namespace wrdac{

        /**
         * @ingroup wrdac
         *
         * Allow retrieving the unique identifier of the item just added
         * to the OPC database through a [add] request.
         *
         * @param reply  the bottle received from the database
         *
         * @return the item identifier
         */
        int opcGetIdFromAdd(yarp::os::Bottle &reply);


        /**
         * @ingroup wrdac
         *
         * Allow retrieving the list of unique identifiers of those items
         * verifying the set of conditions queried to the database
         * through a [ask] request.
         *
         * @param reply  the bottle received from the database
         *
         * @return the list of items identifiers
         */
        yarp::os::Bottle opcGetIdsFromAsk(yarp::os::Bottle &reply);

        /**
         * @ingroup wrdac
         *
         * Simple search and replace function for strings;
         *
         * @param in  the ouput string
         * @param plain  the substring to be replaced
         * @param tok  the replacement
         */
        void replace_all(std::string & in, const std::string & plain, const std::string & tok);

        /**
         * @ingroup wrdac
         *
         * Get the context path of a .grxml grammar, and return it as a string
         *
         * @param sPath Path to the grammar file
         * @return Grammar as string
         */
        std::string grammarToString(std::string sPath);
    }

}
