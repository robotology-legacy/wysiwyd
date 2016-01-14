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

#ifndef __EFAA_SUBSYSTEM_H__
#define __EFAA_SUBSYSTEM_H__

#define SUBSYSTEM               "abstract"

#include <iostream>
#include <iterator>
#include <algorithm>

namespace wysiwyd{
    namespace wrdac{

        //--------------------------------------------------------------------------------------------

        /**
        * \ingroup wrdac_clients
        *
        * Abstract class to handle sub-systems of the architecture (navigation, speech, action, etc...)
        */
        class SubSystem
        {
        protected:
            bool m_isRunning;
            std::string m_masterName;
            std::string m_type;
            virtual bool connect() = 0;

        public:
            SubSystem(const std::string &masterName) { m_isRunning = false; m_masterName = masterName; m_type = SUBSYSTEM; }
            bool isRunning() const { return m_isRunning; }
            bool Connect() { return (m_isRunning = connect()); }
            virtual void Close() = 0;
            std::string getType() const { return m_type; }
        };

    }
}//Namespace
#endif


