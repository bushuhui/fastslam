#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include <string>
#include <vector>

////////////////////////////////////////////////////////////////////////////////
// time functions
////////////////////////////////////////////////////////////////////////////////
uint64_t tm_get_millis(void);
uint64_t tm_get_ms(void);
uint64_t tm_get_us(void);

void   tm_sleep(uint32_t t);


////////////////////////////////////////////////////////////////////////////////
// ASSERT macro
////////////////////////////////////////////////////////////////////////////////
#ifdef RTK_DEBUG

    #define ASSERT(f) \
        do { \
            if (!f ) { \
                fprintf(stderr, "ERROR (FILE: %s, LINE: %d, FUNC: %s)\n", \
                    __FILE__, __LINE__, __FUNCTION__); \
                exit(1); \
            } \
        } while (0); \


    #define ASSERT2(f, s) \
        do { \
            if (!f ) { \
                fprintf(stderr, "ERROR: %s (FILE: %s, LINE: %d, FUNC: %s)\n", \
                    s, __FILE__, __LINE__, __FUNCTION__); \
                exit(1); \
            } \
        } while (0); \

#else
    #define ASSERT(f)
    #define ASSERT2(f, s)
#endif

////////////////////////////////////////////////////////////////////////////////
// debug level
////////////////////////////////////////////////////////////////////////////////
#define RTK_DEBUG_LEVEL_ERROR	1
#define RTK_DEBUG_LEVEL_WARN	2
#define RTK_DEBUG_LEVEL_INFO    3
#define RTK_DEBUG_LEVEL_TRACE	4       // default
#define RTK_DEBUG_LEVEL_NORM	5

// debug level functions
void dbg_set_level(int i);
int  dbg_get_level(void);
void dbg_push_level(int i);
int  dbg_pop_level(void);


////////////////////////////////////////////////////////////////////////////////
// usefull debug print functions
////////////////////////////////////////////////////////////////////////////////

// debug print functions
void dbg_printf(int level,
                const char *fname, int line, const char *func,
                const char *szFmtString, ...);

// level 1: error message
#define dbg_pe(...) dbg_printf(1, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)

// level 2: warning message
#define dbg_pw(...) dbg_printf(2, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)

// level 3: information message (default)
#define dbg_pi(...) dbg_printf(3, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)

// level 4: trace message
#define dbg_pt(...) dbg_printf(4, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)

// level 5: normal message
#define dbg_pn(...) dbg_printf(5, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)


////////////////////////////////////////////////////////////////////////////////
// debug stack trace functions
////////////////////////////////////////////////////////////////////////////////
void dbg_stacktrace_setup(void);



////////////////////////////////////////////////////////////////////////////////
// string functions
////////////////////////////////////////////////////////////////////////////////
typedef std::vector<std::string> StringArray;

// split given string by delims
StringArray split_text(const std::string &intext, const std::string &delims);

// string trim functions
std::string ltrim(const std::string &s);
std::string rtrim(const std::string &s);
std::string trim(const std::string &s);

// string lower & upper
std::string str_tolower(std::string &s);
std::string str_toupper(std::string &s);

// string to int, float, double
int    str_to_int(const std::string &s);
float  str_to_float(const std::string &s);
double str_to_double(const std::string &s);

////////////////////////////////////////////////////////////////////////////////
// arguments functions
////////////////////////////////////////////////////////////////////////////////
void save_arguments(int argc, char *argv[], std::string fname);

////////////////////////////////////////////////////////////////////////////////
// file & path functions
////////////////////////////////////////////////////////////////////////////////
long filelength(FILE *fp);
long filelength(const char *fname);

int path_exist(const char *p);
int path_mkdir(const char *p);

int path_delfile(const std::string &p);

int path_lsdir(const std::string &dir_name, StringArray &dl);
int path_isdir(const std::string &p);
int path_isfile(const std::string &p);

// split path & file name
StringArray path_split(const std::string &fname);
StringArray path_splitext(const std::string &fname);

std::string path_join(const std::string &p1, const std::string &p2);
std::string path_join(const std::string &p1, const std::string &p2, const std::string &p3);
std::string path_join(const StringArray &p);

////////////////////////////////////////////////////////////////////////////////
// text file functions
////////////////////////////////////////////////////////////////////////////////
int readlines(const std::string &fn, StringArray &lns);

////////////////////////////////////////////////////////////////////////////////
// memory function
////////////////////////////////////////////////////////////////////////////////
void memcpy_fast(void *dst, void *src, uint32_t s);



/*****************************************************************************\
\*****************************************************************************/
enum CVariantType
{
    VT_INT,                 // integer value
    VT_FLOAT,               // float value
    VT_DOUBLE,              // double value
    VT_STRING,              // string value
    VT_POINTER,             // pointer value
    VT_BIN,                 // binary value
};

union CVariantUnion
{
    int     iVal;           // integer value
    float   fVal;           // float value
    double  dVal;           // double value
    char    *sVal;          // string value
    void    *pVal;          // pointer value
    void    *bVal;          // binary value
};

class CVariant
{
public:
    // set value
    void set(int        v);
    void set(float      v);
    void set(double     v);
    void set(char       *v);
    void set(const char *v);
    void set(void       *v);

    // get/convert type
    int     to_i(void);
    float   to_f(void);
    double  to_d(void);
    char    *to_s(char *buf);
    char    *to_s(void);
    void    *to_p(void);

    // get length
    int     size(void);

    // to/from stream data
    int  stream_len(void);
    int  to_stream(int *len, uint8_t *buf);
    int  from_stream(int len, uint8_t *buf);

    // assignment operator
    CVariant& operator =(const CVariant &o);

public:
    CVariant()          { _init(); }
    virtual ~CVariant() { _release(); }

private:
    void _init(void);
    void _release(void);

protected:
    CVariantType    t;
    CVariantUnion   d;

    char            *buf;
};

/*****************************************************************************\
\*****************************************************************************/
class CParamArray
{
public:
    CParamArray() { _init(); }
    virtual ~CParamArray() { _release(1); }

    // get value (if exist)
    int i(const std::string &n, int &v);
    int f(const std::string &n, float &v);
    int d(const std::string &n, double &v);
    int s(const std::string &n, std::string &v);
    int p(const std::string &n, void **v);

    // get value
    int         i(const std::string &n);
    float       f(const std::string &n);
    double      d(const std::string &n);
    std::string s(const std::string &n);
    void*       p(const std::string &n);

    // get key exist
    int  key_exist(const std::string &n);

    // set value
    int  set_i(const std::string &n, int v);
    int  set_f(const std::string &n, float v);
    int  set_d(const std::string &n, double v);
    int  set_s(const std::string &n, std::string v);
    int  set_p(const std::string &n, void *v);

    // if not exist, set value
    // if exist, get value
    int  setget_i(const std::string &n, int &v);
    int  setget_f(const std::string &n, float &v);
    int  setget_d(const std::string &n, double &v);
    int  setget_s(const std::string &n, std::string &v);
    int  setget_p(const std::string &n, void **v);

    // parse arguments
    int  set_args(int argc, char *argv[]);

    // load / save
    virtual int load(const std::string &f);
    virtual int save(const std::string &f);

    // to/from stream data
    int stream_len(void);
    int to_stream(int *len, uint8_t *buf);
    virtual int from_stream(int len, uint8_t *buf);

    // parse item
    virtual int parse(void);

    // push/pop settings
    int push(void);
    int pop(void);

    // print parameters
    void print(void);

    // clear all items
    void clear(void);

    // assignment operator
    CParamArray& operator =(const CParamArray &o);

protected:
    std::vector<std::string>    na;     // name array
    std::vector<CVariant*>      va;     // value array
    std::vector<uint8_t*>       sa;     // stack array

    int _init(void);
    int _release(int s);
};

/*****************************************************************************\
\*****************************************************************************/
class CArgs
{
public:
    CArgs() { _init(); }
    virtual ~CArgs() { _release(); }

    // set arguments
    int set_args(int argc, char *argv[]);

    // save arguments to file
    int save(std::string fname);

protected:
    std::vector<std::string>    na;     // argument array

    int _init(void);
    int _release(void);
};



////////////////////////////////////////////////////////////////////////////////
// test module functions
////////////////////////////////////////////////////////////////////////////////
typedef int (*RTK_FUNC_TEST)(CParamArray *pa);

struct RTK_TestFunctionArray {
    RTK_FUNC_TEST       f;
    char                name[200];
    char                desc[200];
};

#define RTK_FUNC_TEST_DEF(f,d) {f,#f,d}

int rtk_test_main(int argc, char *argv[], RTK_TestFunctionArray fa[], CParamArray &pa);


#endif // __UTILS_H__
