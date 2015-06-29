/* 
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Stéphane Lallée
 * email:   stephane.lallee@gmail.com
 * website: http://efaa.upf.edu/ 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * wysiwyd/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */


#include <yarp/os/Vocab.h>
#include <wrdac/functions.h>

using namespace yarp::os;
using namespace wysiwyd::wrdac;

/************************************************************************/
int wysiwyd::wrdac::opcGetIdFromAdd(Bottle &reply)
{
    if (reply.size()>0)
        if (reply.get(0).asVocab()==Vocab::encode("ack"))
            if (Bottle *idProp=reply.get(1).asList())
                return idProp->get(1).asInt();

    return EFAA_OPC_INVALID_ID;
}


/************************************************************************/
Bottle wysiwyd::wrdac::opcGetIdsFromAsk(Bottle &reply)
{
    Bottle ids;
    if (reply.size()>0)
        if (reply.get(0).asVocab()==Vocab::encode("ack"))
            if (Bottle *idProp=reply.get(1).asList())
                if (Bottle *idList=idProp->get(1).asList())
                    ids=*idList;

    return ids;
}
/************************************************************************/
void wysiwyd::wrdac::replace_all(std::string & in, const std::string & plain, const std::string & tok)
{
    std::string::size_type n = 0;
    const std::string::size_type l = plain.length();
    while(1){
        n = in.find(plain, n);
        if(n != std::string::npos){
            in.replace(n, l, tok);
        }
        else{
            break;
        }
    }
}
