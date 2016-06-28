#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>

#include <signal.h>
#include <execinfo.h>
#include <errno.h>
#include <cxxabi.h>
#include <unistd.h>
#include <fcntl.h>
#include <limits.h>
#include <dirent.h>

#include <sys/time.h>
#include <sys/timeb.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>

#include <math.h>
#include <complex.h>
#include <float.h>

#include <assert.h>
#include <inttypes.h>
#include <tmmintrin.h>

#include <string>
#include <vector>
#include <algorithm>

#include "utils.h"


using namespace std;


////////////////////////////////////////////////////////////////////////////////
// time functions
////////////////////////////////////////////////////////////////////////////////

uint64_t tm_get_millis(void)
{
    struct timeval  tm_val;
    uint64_t       v;
    int             ret;

    ret = gettimeofday(&tm_val, NULL);

    v = tm_val.tv_sec*1000 + tm_val.tv_usec/1000;
    return v;
}

uint64_t tm_get_ms(void)
{
    struct timeval  tm_val;
    uint64_t       v;
    int             ret;

    ret = gettimeofday(&tm_val, NULL);

    v = tm_val.tv_sec*1000 + tm_val.tv_usec/1000;
    return v;
}

uint64_t tm_get_us(void)
{
    struct timeval  tm_val;
    uint64_t       v;
    int             ret;

    ret = gettimeofday(&tm_val, NULL);

    v = tm_val.tv_sec*1000000 + tm_val.tv_usec;
    return v;
}

void   tm_sleep(uint32_t t)
{
    struct timespec tp;

    tp.tv_sec = t / 1000;
    tp.tv_nsec = ( t % 1000 ) * 1000000;

    nanosleep(&tp, &tp);
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


// debug level
static int  g_iDebugLevel = 4;

// debug level stack
#define DEBUG_LEVE_STACK_SIZE 128

static int  g_aDebugLevelStack[DEBUG_LEVE_STACK_SIZE];
static int  g_iDebugLevelStackIdx=0;


/**
 *  Set level of debug print
 *
 *  Parameters:
 *      \param[in]  i       level
 */
void dbg_set_level(int i)
{
    g_iDebugLevel = i;
}

/**
 *  Get current debug level
 *
 *  Return value:
 *      current debug level
 */
int  dbg_get_level(void)
{
    return g_iDebugLevel;
}

/**
 *  Push a new debug level to stack
 *
 *  FIXME:
 *      Not implementated yet
 *
 *  Paramters:
 *      \param[in]  i       new debug level
 *  Return value:
 *      None
 */
void dbg_push_level(int level)
{
    if( g_iDebugLevelStackIdx >= DEBUG_LEVE_STACK_SIZE ) {
        dbg_pe("Debug level stack overfull!");
        return;
    }

    g_aDebugLevelStack[g_iDebugLevelStackIdx++] = g_iDebugLevel;
    g_iDebugLevel = level;
}

/**
 *  Pop top debug level from stack
 *
 *  FIXME:
 *      Not implementated yet
 *
 *  Paramters:
 *      None
 *  Return value:
 *      new debug level
 */
int  dbg_pop_level(void)
{
    if( g_iDebugLevelStackIdx <= 0 ) {
        dbg_pe("Debug level stack is empty!");
        return g_iDebugLevel;
    }

    g_iDebugLevel = g_aDebugLevelStack[--g_iDebugLevelStackIdx];
    return g_iDebugLevel;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

char *_str_cat(char *s_out, const char *s1, const char *s2, const char *s3)
{
    int     i, j, l1, l2, l3;

    l1 = strlen(s1);
    l2 = strlen(s2);
    l3 = strlen(s3);

    j = 0;
    for(i=0; i<l1; i++) s_out[j++] = s1[i];
    for(i=0; i<l2; i++) s_out[j++] = s2[i];
    for(i=0; i<l3; i++) s_out[j++] = s3[i];
    s_out[j] = 0;

    return s_out;
}

void dbg_printf(int level,
               const char *fname, int line, const char *func,
               const char *szFmtString, ...)
{
    #define MAX_DBUG_BUFF_LEN    4096

    char    *sHeader, *sTail;
    char    *sBuf1, *sBuf2;
    int     lBuf1, lBuf2;

    va_list va_params;

    // check debug level
    if( level > g_iDebugLevel ) return;

    // alloc string buffer
    lBuf1 = strlen(szFmtString);

    sHeader = new char[MAX_DBUG_BUFF_LEN];
    sTail   = new char[MAX_DBUG_BUFF_LEN];
    sBuf1   = new char[MAX_DBUG_BUFF_LEN+lBuf1];
    sBuf2   = new char[MAX_DBUG_BUFF_LEN+lBuf1];

    // generate header, tail
    if( level == 1 ) {
        sprintf(sHeader, "\033[31mERR:\033[0m  ");
        sprintf(sTail,   "      \033[35m(LINE: %5d, FILE: %s, FUNC: %s)\033[0m", line, fname, func);
    } else if (level == 2 ) {
        sprintf(sHeader, "\033[33mWARN:\033[0m ");
        sprintf(sTail,   "      \033[35m(LINE: %5d, FILE: %s, FUNC: %s)\033[0m", line, fname, func);
    } else if (level == 3 ) {
        sprintf(sHeader, "\033[36mINFO:\033[0m ");
        sprintf(sTail,   "      \033[35m(LINE: %5d, FILE: %s, FUNC: %s)\033[0m", line, fname, func);
    } else if (level == 4 ) {
        sprintf(sHeader, "\033[34m%s\033[0m >> ", func);
        sprintf(sTail, "");
    } else {
        sprintf(sHeader, "");
        sprintf(sTail, "");
    }

    // generate format string
    va_start(va_params, szFmtString);
    vsprintf(sBuf1, szFmtString, va_params);
    va_end(va_params);

    lBuf1 = strlen(sBuf1);
    if( lBuf1 > 0 )
        if( sBuf1[lBuf1-1] != '\n' ) {
            sBuf1[lBuf1] = '\n';
            sBuf1[lBuf1+1] = 0;
        }

    // concatenate final string
    _str_cat(sBuf2, sHeader, sBuf1, sTail);

    lBuf2 = strlen(sBuf2);
    if( lBuf2 > 0 )
        if( sBuf2[lBuf2-1] == '\n' ) {
            sBuf2[lBuf2-1] = 0;
        }

    // output message
    puts(sBuf2);

    // free tem buffer
    delete sHeader;
    delete sTail;
    delete sBuf1;
    delete sBuf2;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

pid_t popen2(const char *command, int *infp, int *outfp)
{
    #define READ    0
    #define WRITE   1

    int p_stdin[2], p_stdout[2];
    pid_t pid;

    if (pipe(p_stdin) != 0 || pipe(p_stdout) != 0)
        return -1;

    pid = fork();

    if (pid < 0)
        return pid;
    else if (pid == 0) {
        close(p_stdin[WRITE]);
        dup2(p_stdin[READ], READ);
        close(p_stdout[READ]);
        dup2(p_stdout[WRITE], WRITE);

        execl("/bin/sh", "sh", "-c", command, NULL);
        perror("execl");
        exit(1);
    }

    if (infp == NULL)
        close(p_stdin[WRITE]);
    else
        *infp = p_stdin[WRITE];

    if (outfp == NULL)
        close(p_stdout[READ]);
    else
        *outfp = p_stdout[READ];

    return pid;
}

int get_exec_output(char *cmd, char *buf, int buf_len)
{
    pid_t   pid;
    int     stat_loc;
    int     infp, outfp;
    int     i, l;
    ssize_t nread;

    // open pipe
    pid = popen2(cmd, &infp, &outfp);
    if ( pid <= 0) {
        printf("ERR: Unable to exec given program\n");
        exit(1);
    }

    // read output
    nread = read(outfp, buf, buf_len);

    // close pip file
    close(infp);
    close(outfp);

    // wait child process finished
    wait(&stat_loc);

    // process output buffer
    l = strlen(buf);

    // get first line
    for(i=0; i<l; i++) {
        if( buf[i] == '\n' || buf[i] == '\r' )
            buf[i] = '\0';
    }

    // normal method remove trailing \r
    /*
    if( buf[l-1] == '\n' || buf[l-1] == '\r' )
        buf[l-1] = '\0';
    */

    return 0;
}

static inline void printStackTrace( FILE *out = stderr, unsigned int max_frames = 100 )
{
    fprintf(out, "stack trace:\n");

    // storage array for stack trace address data
    void    *addrlist[max_frames+1];

    size_t  funcnamesize = 1024;
    char    funcname[1024];
    char    s_addr[200];
    char    s_off[200];
    char    s_cmd[1024];
    char    s_fileline[1024];

    unsigned int    i;

    // retrieve current stack addresses
    unsigned int addrlen = backtrace( addrlist, sizeof( addrlist ) / sizeof( void* ));

    if ( addrlen == 0 ) {
        fprintf( out, "  \n" );
        return;
    }

    // resolve addresses into strings containing "filename(function+address)",
    // Actually it will be ## program address function + offset
    // this array must be free()-ed
    char** symbollist = backtrace_symbols( addrlist, addrlen );


    // iterate over the returned symbol lines. skip the first 3 line, last line
    //      it is the address of this function.
    for ( i = 3; i < addrlen-1; i++ ) {
        char* begin_name   = NULL;
        char* begin_offset = NULL;
        char* end_offset   = NULL;

        // find parentheses and +address offset surrounding the mangled name
#ifdef DARWIN
        // OSX style stack trace
        for ( char *p = symbollist[i]; *p; ++p ) {
            if (( *p == '_' ) && ( *(p-1) == ' ' ))
                begin_name = p-1;
            else if ( *p == '+' )
                begin_offset = p-1;
        }

        if ( begin_name && begin_offset && ( begin_name < begin_offset )) {
            *begin_name++ = '\0';
            *begin_offset++ = '\0';

            // mangled name is now in [begin_name, begin_offset) and caller
            // offset in [begin_offset, end_offset). now apply
            // __cxa_demangle():
            int status;
            char* ret = abi::__cxa_demangle( begin_name, &funcname[0],
                    &funcnamesize, &status );
            if ( status == 0 ) {
                funcname = ret; // use possibly realloc()-ed string
                fprintf( out, "  %-30s %-40s %s\n",
                         symbollist[i], funcname, begin_offset );
            } else {
                // demangling failed. Output function name as a C function with
                // no arguments.
                fprintf( out, "  %-30s %-38s() %s\n",
                         symbollist[i], begin_name, begin_offset );
            }

#else // !DARWIN - but is posix


        // not OSX style

        s_addr[0] = '\0';
        s_off[0]  = '\0';

        // ./module(function+0x15c) [0x8048a6d]
        for ( char *p = symbollist[i]; *p; ++p ) {
            if ( *p == '(' )
                begin_name = p;
            else if ( *p == '+' )
                begin_offset = p;
            else if ( *p == '[' && ( begin_offset || begin_name ) )
                end_offset = p;
        }

        // get address string
        if ( end_offset ) {
            for(char *p=end_offset, j=0; *p; ++p) {
                if( *p == '[' )
                    continue;
                else if( *p == ']' )
                    s_addr[j] = '\0';
                else
                    s_addr[j++] = *p;
            }
            //fprintf(out, "addr: %s\n", s_addr);
        }

        // get offset address
        if( begin_offset ) {
            for(char *p=begin_offset, j=0; *p; ++p) {
                if( *p == '+' )
                    continue;
                else if( *p == ')' ) {
                    s_off[j] = '\0';
                    break;
                } else
                    s_off[j++] = *p;
            }
            //fprintf(out, "offset: %s\n", s_off);
        }

        if ( begin_name && end_offset && ( begin_name < end_offset )) {
            *begin_name++   = '\0';
            *end_offset++   = '\0';

            if ( begin_offset )
                *begin_offset++ = '\0';

            // mangled name is now in [begin_name, begin_offset) and caller
            // offset in [begin_offset, end_offset). now apply
            // __cxa_demangle():

            int status = 0;
            char* ret = abi::__cxa_demangle( begin_name, funcname,
                                             &funcnamesize, &status );
            char* fname = begin_name;
            if ( status == 0 )
                fname = ret;

            if ( begin_offset ) {
                fprintf( out, "  %s [ \033[31m%s\033[0m + %s ] [%s]\n",
                         symbollist[i], fname, s_off, s_addr );
            } else {
                fprintf( out, "  %s [ %s   %s ] [%s]\n",
                         symbollist[i], fname, "", s_addr );
            }

            // print source file and line no.
            sprintf(s_cmd, "addr2line -e %s %s", symbollist[i], s_addr);
            get_exec_output(s_cmd, s_fileline, 1024);
            fprintf(out, "      \033[32m%s\033[0m\n", s_fileline);

#endif  // !DARWIN - but is posix
        } else {
            // couldn't parse the line? print the whole line.
            fprintf(out, "  %s\n", symbollist[i]);
        }
    }

    free(symbollist);
}

void abortHandler( int signum )
{
    // associate each signal with a signal name string.
    const char* name = NULL;

    // get signal name
    switch( signum )
    {
    case SIGABRT: name = "SIGABRT";  break;
    case SIGSEGV: name = "SIGSEGV";  break;
    case SIGBUS:  name = "SIGBUS";   break;
    case SIGILL:  name = "SIGILL";   break;
    case SIGFPE:  name = "SIGFPE";   break;
    }

    // notify the user which signal was caught. We use printf, because this is the most
    // basic output function. Once you get a crash, it is possible that more complex output
    // systems like streams and the like may be corrupted. So we make the most basic call
    // possible to the lowest level, most standard print function.
    printf("\n");
    printf("-------------------------------------------------------------------------------------------\n");
    if ( name )
        fprintf( stderr, "Caught signal %d (%s)\n", signum, name );
    else
        fprintf( stderr, "Caught signal %d\n", signum );

    // Dump a stack trace
    printStackTrace();

    printf("-------------------------------------------------------------------------------------------\n");

    // If you caught one of the above signals, it is likely you just
    // want to quit your program right now.
    exit( signum );
}


void dbg_stacktrace_setup(void)
{
    signal( SIGABRT, abortHandler );
    signal( SIGSEGV, abortHandler );
    signal( SIGBUS,  abortHandler );
    signal( SIGILL,  abortHandler );
    signal( SIGFPE,  abortHandler );
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// string functions
////////////////////////////////////////////////////////////////////////////////

// split given string by delims
StringArray split_text(const string &intext, const string &delims)
{
    StringArray         r;
    string::size_type   begIdx, endIdx;
    string              s;

    begIdx = intext.find_first_not_of(delims);

    while(begIdx != string::npos) {
        // search end of the word
        endIdx = intext.find_first_of(delims, begIdx);
        if( endIdx == string::npos )
            endIdx = intext.length();

        // get the sub string
        s = intext.substr(begIdx, endIdx-begIdx);
        r.push_back(s);

        // find next begin position
        begIdx = intext.find_first_not_of(delims, endIdx);
    }

    return r;
}

// string trim functions
string ltrim(const string &s)
{
    string              delims = " \t\n\r",
                        r;
    string::size_type   i;

    i = s.find_first_not_of(delims);
    if( i == string::npos )
        r = "";
    else
        r = s.substr(i, s.size() - i);

    return r;
}


string rtrim(const string &s)
{
    string              delims = " \t\n\r",
                        r;
    string::size_type   i;

    i = s.find_last_not_of(delims);
    if( i == string::npos )
        r = "";
    else
        r = s.substr(0, i+1);

    return r;
}


string trim(const string &s)
{
    string              delims = " \t\n\r",
                        r;
    string::size_type   i, j;

    i = s.find_first_not_of(delims);
    j = s.find_last_not_of(delims);

    if( i == string::npos ) {
        r = "";
        return r;
    }

    if( j == string::npos ) {
        r = "";
        return r;
    }

    r = s.substr(i, j-i+1);
    return r;
}

string str_tolower(string &s)
{
    for(unsigned long i=0; i < s.size(); i++) {
        s[i] = tolower(s[i]);
    }

    return s;
}

string str_toupper(string &s)
{
    for(unsigned long i=0; i < s.size(); i++) {
        s[i] = toupper(s[i]);
    }

    return s;
}


int str_to_int(const string &s)
{
    return atoi(s.c_str());
}

float str_to_float(const string &s)
{
    return atof(s.c_str());
}

double str_to_double(const string &s)
{
    return atof(s.c_str());
}

////////////////////////////////////////////////////////////////////////////////
// arguments functions
////////////////////////////////////////////////////////////////////////////////

void save_arguments(int argc, char *argv[], string fname)
{
    string      fn;
    FILE        *fp;
    int         i;
    tm          *now;
    time_t      t;
    char        str_time[200];


    fn = fname + "_args.txt";
    fp = fopen(fn.c_str(), "a+"); ASSERT(fp);

    // get current time
    time(&t);
    now = localtime(&t);
    strftime(str_time, 200, "%Y-%m-%d %H:%M:%S", now);

    fprintf(fp, "--------------- %s ---------------\n", str_time);

    for(i=0; i<argc; i++)
        fprintf(fp, "%s ", argv[i]);

    fprintf(fp, "\n\n");

    fclose(fp);
}

////////////////////////////////////////////////////////////////////////////////
// file & path functions
////////////////////////////////////////////////////////////////////////////////

// file functions
long filelength(FILE *fp)
{
    long    len;

    if( fp == NULL )
        return 0;

    fseek(fp, 0, SEEK_END);
    len = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    return len;
}

long filelength(const char *fname)
{
    FILE    *fp;
    long    len;

    fp = fopen(fname, "r");
    if( fp == NULL )
        return 0;

    fseek(fp, 0, SEEK_END);
    len = ftell(fp);
    fclose(fp);

    return len;
}

int path_exist(const char *p)
{
    struct stat     st;
    int             ret;

    ret = stat(p, &st);
    return ret;
}

int path_mkdir(const char *p)
{
    char            cmd[2048];
    int             ret;

    // FIXME: only support UNIX system
    //ret = mknod(p, S_IFDIR | 0775, 0);
    sprintf(cmd, "mkdir -p '%s'", p);

    ret = system(cmd);
    if( ret != 0 ) ret = -1;

    return ret;
}

int path_delfile(const std::string &p)
{
    char            cmd[2048];
    int             ret;

    // FIXME: only support UNIX system
    sprintf(cmd, "rm -rf '%s'", p.c_str());

    ret = system(cmd);
    if( ret != 0 ) ret = -1;

    return ret;
}


int path_lsdir(const string &dir_name, StringArray &dl)
{
    DIR             *dir;
    struct dirent   *dp;

    // open directory
    dir = opendir(dir_name.c_str());
    if( dir == NULL ) {
        dbg_pe("Failed to open dir: %s\n", dir_name.c_str());
        return -1;
    }

    // get each items
    dl.clear();
    for(dp=readdir(dir); dp!=NULL; dp=readdir(dir)) {
        // skip .
        if( strlen(dp->d_name) == 1 && dp->d_name[0] == '.' )
            continue;

        // skip ..
        if( strlen(dp->d_name) == 2 && dp->d_name[0] == '.' && dp->d_name[1] == '.' )
            continue;

        // add to list
        dl.push_back(dp->d_name);
    }

    closedir(dir);

    // sort all file name
    std::sort(dl.begin(), dl.end());

	return 0;
}

int path_isdir(const std::string &p)
{
    struct stat     st;
    int             ret;

    ret = stat(p.c_str(), &st);
    if( ret == -1 ) {
        dbg_pe("Failed at stat! (%s)", p.c_str());
        return 0;
    }

    if ( (st.st_mode & S_IFMT) == S_IFDIR )
        return 1;
    else
        return 0;
}

int path_isfile(const std::string &p)
{
    struct stat     st;
    int             ret;

    ret = stat(p.c_str(), &st);
    if( ret == -1 ) {
        dbg_pe("Failed at stat! (%s)", p.c_str());
        return 0;
    }

    if ( (st.st_mode & S_IFMT) == S_IFREG )
        return 1;
    else
        return 0;
}

int readlines(const std::string &fn, StringArray &lns)
{
    FILE    *fp=NULL;

    char    *buf;
    int     buf_len;
    string  s;

    // clear old data
    lns.clear();

    // alloc buffer
    buf_len = 8196;
    buf = new char[buf_len];

    // open file
    fp = fopen(fn.c_str(), "r");
    ASSERT(fp);

    while( !feof(fp) ) {
        // read a line
        if( NULL == fgets(buf, buf_len, fp) )
            break;

        // remove blank & CR
        s = trim(buf);

        // skip blank line
        if( s.size() < 1 )
            continue;

        // add to list
        lns.push_back(s);
    }

    // close file
    fclose(fp);

    // free array
    delete buf;

    return 0;
}

StringArray path_split(const string &fname)
{
    size_t      found = -1;
    StringArray r;

    r.clear();

    /* find / or \ */
    found = fname.find_last_of("/\\");

    if( found == string::npos ) {
        r.push_back("");
        r.push_back(fname);
        return r;
    }

    // folder
    r.push_back(fname.substr(0, found));
    // file
    r.push_back(fname.substr(found+1));

    return r;
}

StringArray path_splitext(const string &fname)
{
    size_t      found;
    StringArray r;

    r.clear();

    // find .
    found = fname.find_last_of(".");
    if( found == string::npos ) {
        r.push_back(fname);
        r.push_back("");
        return r;
    }

    // filename
    r.push_back(fname.substr(0, found));
    // extname
    r.push_back(fname.substr(found));

    return r;
}

std::string path_join(const std::string &p1, const std::string &p2)
{
    string      p;
    int         l;

    p = p1;

    l = p.size();
    if( p[l-1] == '/' || p[l-1] == '\\' )
        p = p.substr(0, l-1);

    p = p + "/" + p2;
    return p;
}

std::string path_join(const std::string &p1, const std::string &p2, const std::string &p3)
{
    string      p;

    p = path_join(p1, p2);
    return path_join(p, p3);
}


std::string path_join(const StringArray &p)
{
    unsigned long      i, l;
    string  p_all;

    p_all = "";
    for(i=0; i<p.size(); i++) {
        l = p_all.size();
        if( l>0 ) {
            if( p_all[l-1] == '/' || p_all[l-1] == '\\' )
                p_all = p_all.substr(0, l-1);
        }

        p_all = p_all + "/" + p[i];
    }

    return p_all;
}


////////////////////////////////////////////////////////////////////////////////
/// memory function
////////////////////////////////////////////////////////////////////////////////

void memcpy_fast(void *dst, void *src, uint32_t s)
{
    uint8_t    *_src, *_dst;
    uint8_t    *end;
    uint32_t   _s;

    _src = (uint8_t*) src;
    _dst = (uint8_t*) dst;

    // check memory address is aligned
    assert((uintptr_t)_src % 16 == 0);

    // first part using SSE
    _s = (s/16)*16;
    end = _src + _s;
    for (; _src != end; _src += 16, _dst += 16) {
        _mm_storeu_si128((__m128i *) _dst, _mm_load_si128((__m128i *) _src));
    }

    // remainning part using normal copy
    end = (uint8_t*)src + s;
    for(; _src != end; _src++, _dst++) {
        *_dst = *_src;
    }
}

void memcpy_fast_(void *dst, void *src, uint32_t s)
{
    uint8_t    *p_src, *p_dst;
    uint32_t   n_fast, n_last;

    __m128i *src_vec = (__m128i*) src;
    __m128i *dst_vec = (__m128i*) dst;

    // check memory address is aligned
    assert((uintptr_t)src % 16 == 0);

    n_fast = s/64;
    n_last = s - n_fast*64;

    p_src = (uint8_t*) src;
    p_dst = (uint8_t*) dst;
    p_src += n_fast*64;
    p_dst += n_fast*64;

    // first part using SSE
    while( n_fast-- > 0 ) {
        _mm_storeu_si128(dst_vec,   _mm_load_si128(src_vec));
        _mm_storeu_si128(dst_vec+1, _mm_load_si128(src_vec+1));
        _mm_storeu_si128(dst_vec+2, _mm_load_si128(src_vec+2));
        _mm_storeu_si128(dst_vec+3, _mm_load_si128(src_vec+3));

        dst_vec += 4;
        src_vec += 4;
    }

    // remainning part using normal copy
    while( n_last-- > 0 ) {
        *p_dst = *p_src;
        p_src ++;
        p_dst ++;
    }
}





////////////////////////////////////////////////////////////////////////////////
// test module functions
////////////////////////////////////////////////////////////////////////////////

int rtk_test_default(CParamArray *pa)
{
    printf("default test routine\n");
    return 0;
}

void rtk_print_basic_help(int argc, char *argv[], RTK_TestFunctionArray fa[], CParamArray &pa)
{
    int     i, j, k;
    int     tab = 30;
    char    buf[200];

    printf("\n");
    printf("-------------------------- basic usage --------------------------\n");
    printf("    -f              config file\n");
    printf("    -h              print usage help\n");
    printf("    -dbg_level      [0/1/2/3/4/5] debug level\n");
    printf("                        1 - Error\n");
    printf("                        2 - Warning\n");
    printf("                        3 - Info\n");
    printf("                        4 - Trace\n");
    printf("                        5 - Normal\n");
    printf("    -act            [s] test function name\n");
    printf("\n");

    printf("--------------------------- functions ---------------------------\n");

    i=0;
    while( fa[i].f != NULL ) {
        sprintf(buf, "%s", fa[i].name);
        j = strlen(buf);
        for(k=j; k<tab; k++) buf[k] = 0x20;
        buf[tab] = 0;

        printf("    %s : %s\n", buf, fa[i].desc);
        i++;
    }

    printf("-----------------------------------------------------------------\n");
    printf("\n");
}

int rtk_test_main(int argc, char *argv[], RTK_TestFunctionArray fa[], CParamArray &pa)
{
    string          act;
    int             dbg_level = 4;
    int             i, j=0;
    uint64_t        t1=0, t2=0;
    int             ret=-1;
    string          fname;
    StringArray     sa1, sa2;

    // set signal handler
    dbg_stacktrace_setup();

    // parse input arguments
    if( argc <= 1 ) {
        ret = 0;
        goto RTK_TEST_MAIN_PRINT;
    }

    for(i=0; i<argc; i++) {
        // get config file
        if( strcmp(argv[i], "-f") == 0 ) {
            fname = argv[++i];
        }
        // print usage
        else if( strcmp(argv[i], "-h") == 0 ) {
            ret = 0;
            goto RTK_TEST_MAIN_PRINT;
        }
        // debug level
        else if( strcmp(argv[i], "-dbg_level") == 0 ) {
            dbg_level = atoi(argv[++i]);
            dbg_set_level(dbg_level);
        }
    }

    // load config file
    if( fname.length() > 0 )
        pa.load(fname + ".ini");

    // parse input argument again
    pa.set_args(argc, argv);

    // save input arguments to file
    sa1 = path_split(argv[0]);
    sa2 = path_splitext(sa1[1]);
    save_arguments(argc, argv, sa2[0]);

    // print all settings
    pa.print();

    // test actions
    act = "test_default";
    pa.s("act", act);

    // call test function
    i = 0; j = 0;
    while( fa[i].f != NULL ) {
        if( strcmp(act.c_str(), "test_default") == 0 ) {
            rtk_test_default(&pa);
            break;
        } else if( strcmp(act.c_str(), fa[i].name) == 0 ) {

            // run routine
            t1 = tm_get_ms();
            ret = fa[i].f(&pa);
            t2 = tm_get_ms();
            j = 1;

            break;
        }

        i++;
    }

    if( j == 0 ) {
        printf("ERR: Input argument error!\n\n");
        goto RTK_TEST_MAIN_PRINT;
    }

    // print running time
    printf("\n---------------------------------------------------------\n");
    printf("run time = %g sec (%g min)\n",
                1.0*(t2-t1)/1000.0,         /* sec */
                1.0*(t2-t1)/60000.0);       /* min */
    printf("---------------------------------------------------------\n");

    goto RTK_TEST_MAIN_RET;

RTK_TEST_MAIN_PRINT:
    // print basic arguments
    rtk_print_basic_help(argc, argv, fa, pa);

    // print user provided help
    i=0;
    while( fa[i].f != NULL ) {
        if( strcmp(fa[i].name, "print_help") == 0 ) {
            ret = fa[i].f(&pa);
            break;
        }
        i++;
    }

RTK_TEST_MAIN_RET:
    return ret;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


#define PARAM_ARRAY_ITEM_MAXLEN     4096

/*****************************************************************************\
\*****************************************************************************/
void CVariant::set(int v)
{
    if( t == VT_STRING ) delete d.sVal;

    t = VT_INT;
    d.iVal = v;
}

void CVariant::set(float  v)
{
    if( t == VT_STRING ) delete d.sVal;

    t = VT_FLOAT;
    d.fVal = v;
}

void CVariant::set(double v)
{
    if( t == VT_STRING ) delete d.sVal;

    t = VT_DOUBLE;
    d.dVal = v;
}

void CVariant::set(char *v)
{
    int     n;

    if( t == VT_STRING ) delete d.sVal;

    t = VT_STRING;
    n = strlen(v);
    d.sVal = new char[n+1];
    strcpy(d.sVal, v);
}

void CVariant::set(const char *v)
{
    set((char*) v);
}

void CVariant::set(void *v)
{
    if( t == VT_STRING ) delete d.sVal;

    t = VT_POINTER;
    d.pVal = v;
}

int CVariant::to_i(void)
{
    switch(t) {
    case VT_INT:
        return d.iVal;
        break;

    case VT_FLOAT:
        return (int)(d.fVal);
        break;

    case VT_DOUBLE:
        return (int)(d.dVal);
        break;

    case VT_STRING:
        return atoi(d.sVal);
        break;

    case VT_POINTER:
        return 0;
        break;
    }

    return 0;
}

float CVariant::to_f(void)
{
    switch(t) {
    case VT_INT:
        return (float)(d.iVal);
        break;

    case VT_FLOAT:
        return d.fVal;
        break;

    case VT_DOUBLE:
        return (float)(d.dVal);
        break;

    case VT_STRING:
        return (float)(atof(d.sVal));
        break;

    case VT_POINTER:
        return 0;
        break;
    }

    return 0;
}

double CVariant::to_d(void)
{
    switch(t) {
    case VT_INT:
        return (double)(d.iVal);
        break;

    case VT_FLOAT:
        return (double)(d.fVal);
        break;

    case VT_DOUBLE:
        return d.dVal;
        break;

    case VT_STRING:
        return (double)(atof(d.sVal));
        break;

    case VT_POINTER:
        return 0;
        break;
    }

    return 0;
}

char *CVariant::to_s(char *buf)
{
    buf[0] = 0;

    switch(t) {
    case VT_INT:
        sprintf(buf, "%d", d.iVal);
        break;

    case VT_FLOAT:
        sprintf(buf, "%g", d.fVal);
        break;

    case VT_DOUBLE:
        sprintf(buf, "%g", d.dVal);
        break;

    case VT_STRING:
        return d.sVal;
        break;

    case VT_POINTER:
        // FIXME: change to u64
        sprintf(buf, "%lx", (uint64_t) d.pVal);
        break;
    }

    return buf;
}

char *CVariant::to_s(void)
{
    // FIXME: use a fixed length
    if( buf == NULL ) {
        buf = new char[PARAM_ARRAY_ITEM_MAXLEN];
    }

    return to_s(buf);
}

void *CVariant::to_p(void)
{
    if( t == VT_POINTER )
        return d.pVal;
    else
        return NULL;
}

int CVariant::size(void)
{
    switch(t) {
    case VT_INT:
        return sizeof(int);
        break;

    case VT_FLOAT:
        return sizeof(float);
        break;

    case VT_DOUBLE:
        return sizeof(double);
        break;

    case VT_STRING:
        return strlen(d.sVal)+1;
        break;

    case VT_POINTER:
        return sizeof(void *);
        break;
    }

    return 0;
}

int  CVariant::stream_len(void)
{
    int     l1, l2;

    l1   = sizeof(CVariantType);
    l2   = size();
    return (l1+l2);
}

/**
 *  to stream data
 *
 *  Parameters:
 *      \param[out]     len         data length
 *      \param[out]     buf         data buff
 *  Return Value:
 *      0               successfule
 */
int  CVariant::to_stream(int *len, uint8_t *buf)
{
    int     i, l1, l2;

    l1   = sizeof(CVariantType);
    l2   = size();
    *len = l1 + l2;

    // copy data type field
    i = 0;
    memcpy(buf+i, &t, sizeof(int));
    i += l1;

    // copy data
    switch(t) {
    case VT_INT:
        memcpy(buf+i, &(d.iVal), sizeof(int));
        break;

    case VT_FLOAT:
        memcpy(buf+i, &(d.fVal), sizeof(float));
        break;

    case VT_DOUBLE:
        memcpy(buf+i, &(d.dVal), sizeof(double));
        break;

    case VT_STRING:
        memcpy(buf+i, d.sVal, l2);
        break;

    case VT_POINTER:
        memcpy(buf+i, &(d.pVal), l2);
        break;
    }

    return 0;
}

int  CVariant::from_stream(int len, uint8_t *buf)
{
    int     i, l1, l2;

    l1   = sizeof(CVariantType);
    l2   = len - l1;

    // copy data type field
    i = 0;
    memcpy(&t, buf+i, sizeof(int));
    i += l1;

    // copy data
    switch(t) {
    case VT_INT:
        memcpy(&(d.iVal), buf+i, sizeof(int));
        break;

    case VT_FLOAT:
        memcpy(&(d.fVal), buf+i, sizeof(float));
        break;

    case VT_DOUBLE:
        memcpy(&(d.dVal), buf+i, sizeof(double));
        break;

    case VT_STRING:
        if( d.sVal != NULL ) delete d.sVal;
        d.sVal = new char[l2+1];
        memcpy(d.sVal, buf+i, l2);
        break;

    case VT_POINTER:
        memcpy(&(d.pVal), buf+i, l2);
        break;
    }

    return 0;
}

CVariant& CVariant::operator =(const CVariant &o)
{
    if( this == &o ) return *this;

    if( o.t == VT_STRING ) {
        int l;
        l = strlen(o.d.sVal) + 1;

        if( t == VT_STRING ) delete d.sVal;
        d.sVal = new char[l];

        strcpy(d.sVal, o.d.sVal);
        t = o.t;
    } else {
        t = o.t;
        d = o.d;
    }

    return *this;
}

void CVariant::_init(void)
{
    t = VT_INT;
    memset(&d, 0, sizeof(CVariantUnion));

    buf = NULL;
}

void CVariant::_release(void)
{
    if( t == VT_STRING ) {
        delete d.sVal;
        t = VT_INT;
        d.iVal = 0;
    }

    if( buf != NULL ) {
        delete buf;
        buf = NULL;
    }
}

/*****************************************************************************\
\*****************************************************************************/

int CParamArray::load(const string &f)
{
    FILE            *fp=NULL;
    char            *buf;
    StringArray     sa;
    string          _b;

    string          _n;
    CVariant        *_v;

    // open parameter array file
    fp = fopen(f.c_str(), "rt");
    if( fp == NULL ) {
        dbg_pe("Failed to open file: %s\n", f.c_str());
        return -1;
    }

    // clear old data
    na.clear();
    va.clear();

    // alloc string buffer for reading file
    buf = (char *) malloc(PARAM_ARRAY_ITEM_MAXLEN);

    while(!feof(fp)) {
        // read a line
        if( NULL == fgets(buf, PARAM_ARRAY_ITEM_MAXLEN, fp) )
            break;

        // remove blank & CR
        _b = trim(buf);

        if( _b.size() < 1 )
            continue;

        // skip comment
        if( _b[0] == '#' || _b[0] == ':' )
            continue;

        // FIXME: if current line have more than one of "="
        //        then it will be failed
        sa = split_text(_b, "=");

        if( sa.size() >= 2 ) {
            _n = trim(sa[0]);

            _v = new CVariant;
            _v->set(trim(sa[1]).c_str());

            na.push_back(_n);
            va.push_back(_v);
        }
    }

    // free file & buf
    free(buf);
    fclose(fp);

    // parse items
    parse();

    return 0;
}

int CParamArray::save(const string &f)
{
    return 0;
}

int CParamArray::i(const string &n, int &v)
{
    int     i, l;

    l = na.size();
    for(i=0; i<l; i++) {
        if( n == na[i] ) {
            try {
                v = va[i]->to_i();
            } catch(...) {
                return -1;
            }

            return 0;
        }
    }

    return -1;
}

int CParamArray::i(const string &n)
{
    int     i, l;

    l = na.size();
    for(i=0; i<l; i++) {
        if( n == na[i] ) {
            int _v;

            try {
                _v = va[i]->to_i();
            } catch(...) {
                ASSERT2(0, "CParmArray::i >> cann't conver value");
            }

            return _v;
        }
    }

    ASSERT2(0, "CParmArray::i >> cann't conver value");
    return 0;
}



int CParamArray::f(const string &n, float &v)
{
    int     i, l;

    l = na.size();
    for(i=0; i<l; i++) {
        if( n == na[i] ) {
            try {
                v = va[i]->to_f();
            } catch(...)  {
                return -1;
            }

            return 0;
        }
    }

    return -1;
}

float CParamArray::f(const string &n)
{
    int     i, l;

    l = na.size();
    for(i=0; i<l; i++) {
        if( n == na[i] ) {
            float _v;

            try {
                _v = va[i]->to_f();
            } catch(...)  {
                ASSERT2(0, "CParmArray::f >> cann't conver value");
            }

            return _v;
        }
    }

    ASSERT2(0, "CParmArray::f >> cann't get value");
    return 0.0;
}


int CParamArray::d(const string &n, double &v)
{
    int     i, l;

    l = na.size();
    for(i=0; i<l; i++) {
        if( n == na[i] ) {
            try {
                v = va[i]->to_d();
            } catch(...) {
                return -1;
            }

            return 0;
        }
    }

    return -1;
}

double CParamArray::d(const string &n)
{
    int     i, l;

    l = na.size();
    for(i=0; i<l; i++) {
        if( n == na[i] ) {
            double _v;

            try {
                _v = va[i]->to_d();
            } catch(...) {
                ASSERT2(0, "CParmArray::d >> cann't conver value");
            }

            return _v;
        }
    }

    ASSERT2(0, "CParmArray::d >> cann't get value");
    return 0.0;
}

int CParamArray::s(const string &n, string &v)
{
    int     i, l;
    string  s;

    l = na.size();
    for(i=0; i<l; i++) {
        if( n == na[i] ) {
            l = va[i]->size();
            s = va[i]->to_s();

            if( s[0] == '\"' && s[l-1] == '\"' )  {
                v = s.substr(1, l-2);
            } else {
                v = s;
            }

            return 0;
        }
    }

    return -1;
}

string CParamArray::s(const string &n)
{
    int     i, l;
    string  s, s2;

    l = na.size();
    for(i=0; i<l; i++) {
        if( n == na[i] ) {

            l = va[i]->size();
            s = va[i]->to_s();

            if( s[0] == '\"' && s[l-1] == '\"' )  {
                s2 = s.substr(1, l-2);
            } else {
                s2 = s;
            }

            return s2;
        }
    }

    ASSERT2(0, "CParmArray::s >> cann't get value");
    return "";
}

int CParamArray::p(const string &n, void **v)
{
    int     i, l;

    l = na.size();
    for(i=0; i<l; i++) {
        if( n == na[i] ) {
            *v = va[i]->to_p();

            return 0;
        }
    }

    return -1;
}

void *CParamArray::p(const string &n)
{
    int     i, l;

    l = na.size();
    for(i=0; i<l; i++) {
        if( n == na[i] ) {
            return va[i]->to_p();
        }
    }

    return NULL;
}


int  CParamArray::key_exist(const string &n)
{
    int         i, l;

    // find given key exist
    l = na.size();
    for(i=0; i<l; i++) {
        if( n == na[i] ) {
            return 1;
        }
    }

    return 0;
}

int  CParamArray::set_i(const string &n, int v)
{
    int         i, l;
    string      _n;
    CVariant    *_v;

    // find given item exist
    l = na.size();
    for(i=0; i<l; i++) {
        if( n == na[i] ) {
            va[i]->set(v);
            return 0;
        }
    }

    // insert new item
    _n = n;
    _v = new CVariant();
    _v->set(v);
    na.push_back(_n);
    va.push_back(_v);

    return 0;
}

int  CParamArray::set_f(const string &n, float v)
{
    int         i, l;
    string      _n;
    CVariant    *_v;

    // find given item exist
    l = na.size();
    for(i=0; i<l; i++) {
        if( n == na[i] ) {
            va[i]->set(v);
            return 0;
        }
    }

    // insert new item
    _n = n;
    _v = new CVariant();
    _v->set(v);
    na.push_back(_n);
    va.push_back(_v);

    return 0;
}

int  CParamArray::set_d(const string &n, double v)
{
    int         i, l;
    string      _n;
    CVariant    *_v;

    // find given item exist
    l = na.size();
    for(i=0; i<l; i++) {
        if( n == na[i] ) {
            va[i]->set(v);
            return 0;
        }
    }

    // insert new item
    _n = n;
    _v = new CVariant();
    _v->set(v);
    na.push_back(_n);
    va.push_back(_v);

    return 0;
}

int  CParamArray::set_s(const string &n, string v)
{
    int         i, l;
    string      _n;
    CVariant    *_v;

    // find item exist
    l = na.size();
    for(i=0; i<l; i++) {
        if( n == na[i] ) {
            va[i]->set(v.c_str());
            return 0;
        }
    }

    // insert new item
    _n = n;
    _v = new CVariant();
    _v->set(v.c_str());
    na.push_back(_n);
    va.push_back(_v);

    return 0;
}

int  CParamArray::set_p(const string &n, void *v)
{
    int         i, l;
    string      _n;
    CVariant    *_v;

    // find item exist
    l = na.size();
    for(i=0; i<l; i++) {
        if( n == na[i] ) {
            va[i]->set(v);
            return 0;
        }
    }

    // insert new item
    _n = n;
    _v = new CVariant();
    _v->set(v);
    na.push_back(_n);
    va.push_back(_v);

    return 0;
}

int  CParamArray::setget_i(const string &n, int &v)
{
    int         i, l;
    string      _n;
    CVariant    *_v;

    // find item exist
    l = na.size();
    for(i=0; i<l; i++) {
        // get value
        if( n == na[i] ) {
            v = va[i]->to_i();
            return 0;
        }
    }

    // set value
    _n = n;
    _v = new CVariant();
    _v->set(v);
    na.push_back(_n);
    va.push_back(_v);

    return 0;
}

int  CParamArray::setget_f(const string &n, float &v)
{
    int         i, l;
    string      _n;
    CVariant    *_v;

    // find item exist
    l = na.size();
    for(i=0; i<l; i++) {
        // get value
        if( n == na[i] ) {
            v = va[i]->to_f();
            return 0;
        }
    }

    // set value
    _n = n;
    _v = new CVariant();
    _v->set(v);
    na.push_back(_n);
    va.push_back(_v);

    return 0;
}

int  CParamArray::setget_d(const string &n, double &v)
{
    int         i, l;
    string      _n;
    CVariant    *_v;

    // find item exist
    l = na.size();
    for(i=0; i<l; i++) {
        // get value
        if( n == na[i] ) {
            v = va[i]->to_d();
            return 0;
        }
    }

    // set value
    _n = n;
    _v = new CVariant();
    _v->set(v);
    na.push_back(_n);
    va.push_back(_v);

    return 0;
}

int  CParamArray::setget_s(const string &n, string &v)
{
    int         i, l;
    string      _n;
    CVariant    *_v;

    // find item exist
    l = na.size();
    for(i=0; i<l; i++) {
        // get value
        if( n == na[i] ) {
            v = va[i]->to_s();
            return 0;
        }
    }

    // set value
    _n = n;
    _v = new CVariant();
    _v->set(v.c_str());
    na.push_back(_n);
    va.push_back(_v);

    return 0;
}

int  CParamArray::setget_p(const string &n, void **v)
{
    int         i, l;
    string      _n;
    CVariant    *_v;

    // find item exist
    l = na.size();
    for(i=0; i<l; i++) {
        // get value
        if( n == na[i] ) {
            *v = va[i]->to_p();
            return 0;
        }
    }

    // set value
    _n = n;
    _v = new CVariant();
    _v->set(*v);
    na.push_back(_n);
    va.push_back(_v);

    return 0;
}

int CParamArray::set_args(int argc, char *argv[])
{
    int     i;
    char    *p;
    string  v;

    for(i=1; i<argc; i++) {
        if( argv[i][0] == '-' ) {
            p = argv[i]+1;
            v = argv[++i];
            set_s(p, v);
        }
    }

    return 0;
}

int  CParamArray::stream_len(void)
{
    int         n, i;
    int         l1, l2, l_all;

    // get item number
    n = na.size();

    // stream length + item number + each item's length
    l_all = sizeof(int) + sizeof(int) + sizeof(int)*n;

    // get each item's length
    for(i=0; i<n; i++) {
        l1 = strlen(na[i].c_str()) + 1;
        l2 = va[i]->stream_len();

        l_all += sizeof(int)+l1+l2;
    }

    return l_all;
}

/**
 *  To stream data buf
 *
 *  Parameters:
 *      \param[out]     len     data length
 *      \param[out]     buf     data stream
 *  Return Value:
 *      0               success
 */
int  CParamArray::to_stream(int *len, uint8_t *buf)
{
    int         n, i;
    int         l1, l2, l_item, p;
    int         *arr_len;

    // get item number
    n = na.size();

    // alloc length array
    arr_len = new int[n];

    // get each item's length
    for(i=0; i<n; i++) {
        l1 = strlen(na[i].c_str()) + 1;
        l2 = va[i]->stream_len();

        arr_len[i] = sizeof(int) + l1 + l2;
    }

    // generate stream
    p = 0;

    // stream total length
    p += sizeof(int);

    // item number
    memcpy(buf+p, &n, sizeof(int));
    p += sizeof(int);

    // each item's length
    for(i=0; i<n; i++) {
        memcpy(buf+p, &(arr_len[i]), sizeof(int));
        p += sizeof(int);
    }

    // for each item
    for(i=0; i<n; i++) {
        // name length
        l1 = strlen(na[i].c_str()) + 1;
        memcpy(buf+p, &l1, sizeof(int));
        p += sizeof(int);

        // name
        memcpy(buf+p, na[i].c_str(), l1);
        p += l1;

        // value
        va[i]->to_stream(&l_item, buf+p);
        p += l_item;
    }

    // set total length
    memcpy(buf, &p, sizeof(int));

    // return length
    *len = p;

    delete arr_len;

    return 0;
}

int  CParamArray::from_stream(int len, uint8_t *buf)
{
    int         n, i;
    int         l1, l2, p;
    char        *str_name;
    int         pi;
    int         *arr_len;
    CVariant    *v;

    // name string
    str_name = new char[PARAM_ARRAY_ITEM_MAXLEN];

    // clear old data
    clear();

    // stream position index
    p = 0;

    // stream total length
    p += sizeof(int);

    // get item number
    memcpy(&n, buf+p, sizeof(int));
    p += sizeof(int);

    // alloc length array
    arr_len = new int[n];

    // get each item's length
    for(i=0; i<n; i++) {
        memcpy(&pi, buf+p, sizeof(int));
        arr_len[i] = pi;
        p += sizeof(int);
    }

    // for each item
    for(i=0; i<n; i++) {
        // name length
        memcpy(&pi, buf+p, sizeof(int));
        l1 = pi;
        p += sizeof(int);

        // name
        memcpy(str_name, buf+p, l1);
        p += l1;

        // value
        l2 = arr_len[i] - sizeof(int) - l1;
        v = new CVariant();
        v->from_stream(l2, buf+p);
        p += l2;

        // add name/value to array
        na.push_back(str_name);
        va.push_back(v);
    }

    // parse items
    parse();

    // free temp variables
    delete str_name;
    delete arr_len;

    return 0;
}

int CParamArray::parse(void)
{
    return 0;
}

int CParamArray::push(void)
{
    int     buf_len;
    uint8_t   *buf;

    // to steam
    buf_len = stream_len();
    buf = new uint8_t[buf_len];
    to_stream(&buf_len, buf);

    // push to stack
    sa.push_back(buf);

    return 0;
}

int CParamArray::pop(void)
{
    int     buf_len;
    uint8_t   *buf;

    if( sa.size() > 0 ) {
        buf = sa.back();

        memcpy(&buf_len, buf, sizeof(int));
        from_stream(buf_len, buf);
        delete buf;

        sa.pop_back();
    }

    return 0;
}

void CParamArray::print(void)
{
    int     i, l, n_l, max_l, max_l_def;
    char    str_fmt[300];

    // item number
    l = na.size();

    // determin max name length
    max_l_def = 10;
    max_l     = max_l_def;
    for(i=0; i<l; i++) {
        n_l = strlen(na[i].c_str());
        if( n_l > max_l ) max_l = n_l;
    }

    if( max_l > max_l_def ) max_l += 2;

    // generate format string
    sprintf(str_fmt, "%%%ds = %%s\n", max_l+2);

    // print
    printf("-------------------- Parameters -------------------------\n");

    for(i=0; i<l; i++)
        printf(str_fmt, na[i].c_str(), va[i]->to_s());

    printf("---------------------------------------------------------\n\n");
}

void CParamArray::clear(void)
{
    _release(0);
}

CParamArray& CParamArray::operator =(const CParamArray &o)
{
    int         i, l;
    CVariant    *v;

    // check if self-assignment
    if( this == &o ) return *this;

    // clear old contents
    clear();

    // get item number
    l = o.na.size();

    // copy each item
    for(i=0; i<l; i++) {
        v = new CVariant;
        *v = *(o.va[i]);

        na.push_back(o.na[i]);
        va.push_back(v);
    }

    // parse some field
    parse();

    return *this;
}

int CParamArray::_init(void)
{
    // clear name & value array
    na.clear();
    va.clear();

    // clear stack array
    sa.clear();

    return 0;
}

int CParamArray::_release(int s)
{
    unsigned long     i, l;

    // free all variant objects
    l = na.size();

    for(i=0; i<l; i++) {
        delete va[i];
    }

    // clear name & value list
    na.clear();
    va.clear();

    // clear stack objects
    if( s == 1 ) {
        for(i=0; i<sa.size(); i++) delete sa[i];
        sa.clear();
    }

    return 0;
}

/*****************************************************************************\
\*****************************************************************************/

int CArgs::_init(void)
{
    na.clear();

    return 0;
}

int CArgs::_release(void)
{
    na.clear();

    return 0;
}

int CArgs::set_args(int argc, char *argv[])
{
    int     i;

    na.clear();

    for(i=0; i<argc; i++) {
        na.push_back(argv[i]);
    }

    return 0;
}

int CArgs::save(string fname)
{
    string      fn;
    FILE        *fp;
    unsigned long i;
    tm          *now;
    time_t      t;
    char        str_time[200];


    fn = fname + "_args.txt";
    fp = fopen(fn.c_str(), "a+");
    ASSERT(fp);

    // get current time
    time(&t);
    now = localtime(&t);
    strftime(str_time, 200, "%Y-%m-%d %H:%M:%S", now);

    fprintf(fp, "--------------- %s ---------------\n", str_time);

    for(i=0; i<na.size(); i++)
        fprintf(fp, "%s ", na[i].c_str());

    fprintf(fp, "\n\n");

    fclose(fp);

    return 0;
}

