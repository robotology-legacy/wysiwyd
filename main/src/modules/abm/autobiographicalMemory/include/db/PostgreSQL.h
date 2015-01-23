//
// PostgreSql.h
//
// PostgreSql imp.
//
// Copyright (c) 2007, Renato Tegon Forti
//
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
// SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
// FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//


#include "Exception.h"
#include "ResultSet.h"
#include "DataBase.h"
#include "libpq-fe.h" // Client programs that use libpq must include the header file libpq-fe.h and must link with the libpq library. 

class PostgreSql
	/// PostgreSql data base class
	/// http://www.postgresql.org/docs/8.3/interactive/libpq.html
{
	friend class DataBase<PostgreSql>;

public:

	PostgreSql()
		: _hasResult(false)
	{
	}

	virtual ~PostgreSql()
	{
	}

	void connect(const std::string& server, const std::string& user, const std::string& password, const std::string& database)
	{
		_connectionHandlerPtr = PQsetdbLogin(
					server.c_str(),
                    NULL,
                    NULL,
                    NULL,
                    database.c_str(),
                    user.c_str(),
                    password.c_str()
					);

		// Check to see that the backend connection was successfully made
		if (PQstatus(_connectionHandlerPtr) != CONNECTION_OK)
		{
			std::string msg("Failed to connect to database: Error: " + std::string(PQerrorMessage(_connectionHandlerPtr)));

			PQfinish(_connectionHandlerPtr);

			throw DataBaseError(msg);
		}

	}

	void execute(const std::string& sql)
	{
		//std::cout << sql << std::endl;

		if(_hasResult)
			PQclear(_resultPtr);

		_resultPtr = PQexec(_connectionHandlerPtr, sql.c_str());

		if(_resultPtr == NULL)
		{
			throw DataBaseError("Failed to execute sql: Error: " + std::string(PQerrorMessage(_connectionHandlerPtr)));
		}

		_status = PQresultStatus(_resultPtr);

		if ((_status == PGRES_EMPTY_QUERY) || (_status == PGRES_BAD_RESPONSE) || (_status == PGRES_FATAL_ERROR))
		{
			PQclear(_resultPtr);

			_hasResult = false;

			throw DataBaseError("Failed to execute sql: Error: "
				+ std::string(PQresStatus(_status))
				+ " : " + std::string(PQerrorMessage(_connectionHandlerPtr)));
		}

		if (_status != PGRES_TUPLES_OK) {
                     PQclear(_resultPtr) ;
                } else {
                    _hasResult = true;}
	}

	void populate(ResultSet& rs)
	{

		if(_status != PGRES_TUPLES_OK)
			throw DataBaseError("This command don't support results");

		if(_hasResult == false)
			throw DataBaseError("Any results available");

		unsigned int num_tuples = PQntuples(_resultPtr);
		unsigned int num_fields = PQnfields(_resultPtr);

		for(unsigned int i = 0; i < num_tuples; ++i)
		{
			std::vector<std::string> myRow;

			for(unsigned int j = 0; j < num_fields; ++j)
			{
				myRow.push_back(PQgetvalue(_resultPtr, i, j));
			}

			rs.addRow(myRow);
		}

		PQclear(_resultPtr);

		_hasResult = false;
	}

    unsigned int lo_import_pgsql(const char *filename) {
        _resultPtr = PQexec(_connectionHandlerPtr, "BEGIN");
        if (PQresultStatus(_resultPtr) != PGRES_COMMAND_OK) {
            throw DataBaseError("BEGIN command failed: " + std::string(PQerrorMessage(_connectionHandlerPtr)));
        }

        int res = lo_import(_connectionHandlerPtr, filename);
        if (res == -1) {
            throw DataBaseError("lo_import failed: " + std::string(PQerrorMessage(_connectionHandlerPtr)));
        }

        PQexec(_connectionHandlerPtr, "END");
        PQclear(_resultPtr);

        return res;
    }

    int	lo_export_pgsql(unsigned int lobjId, const char *filename) {
        _resultPtr = PQexec(_connectionHandlerPtr, "BEGIN");
        if (PQresultStatus(_resultPtr) != PGRES_COMMAND_OK) {
            throw DataBaseError("BEGIN command failed: " + std::string(PQerrorMessage(_connectionHandlerPtr)));
        }
        int res = lo_export(_connectionHandlerPtr, lobjId, filename);

        if (res == -1) {
            throw DataBaseError("lo_export failed: " + std::string(PQerrorMessage(_connectionHandlerPtr)));
        }

        PQexec(_connectionHandlerPtr, "END");
        PQclear(_resultPtr);

        return res;
    }

protected:

	void close(void)
		/// close the connection to the database and cleanup
	{
		PQfinish(_connectionHandlerPtr);
	}


private:
	ExecStatusType	_status;
	bool			_hasResult;
	PGresult*		_resultPtr;
	PGconn*			_connectionHandlerPtr;

}; // PostgreSql
