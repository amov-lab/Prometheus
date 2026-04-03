#ifndef SPIRE_ELLIPSEDETECTOR_H
#define SPIRE_ELLIPSEDETECTOR_H

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <unordered_map>



#ifdef _WIN32
/*
* Define architecture flags so we don't need to include windows.h.
* Avoiding windows.h makes it simpler to use windows sockets in conjunction
* with dirent.h.
*/
#if !defined(_68K_) && !defined(_MPPC_) && !defined(_X86_) && !defined(_IA64_) && !defined(_AMD64_) && defined(_M_IX86)
#   define _X86_
#endif
#if !defined(_68K_) && !defined(_MPPC_) && !defined(_X86_) && !defined(_IA64_) && !defined(_AMD64_) && defined(_M_AMD64)
#define _AMD64_
#endif

#include <stdio.h>
#include <stdarg.h>
#include <windef.h>
#include <winbase.h>
#include <wchar.h>
#include <string.h>
#include <stdlib.h>
#include <malloc.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

/* Indicates that d_type field is available in dirent structure */
#define _DIRENT_HAVE_D_TYPE

/* Indicates that d_namlen field is available in dirent structure */
#define _DIRENT_HAVE_D_NAMLEN

/* Entries missing from MSVC 6.0 */
#if !defined(FILE_ATTRIBUTE_DEVICE)
#   define FILE_ATTRIBUTE_DEVICE 0x40
#endif

/* File type and permission flags for stat(), general mask */
#if !defined(S_IFMT)
#   define S_IFMT _S_IFMT
#endif

/* Directory bit */
#if !defined(S_IFDIR)
#   define S_IFDIR _S_IFDIR
#endif

/* Character device bit */
#if !defined(S_IFCHR)
#   define S_IFCHR _S_IFCHR
#endif

/* Pipe bit */
#if !defined(S_IFFIFO)
#   define S_IFFIFO _S_IFFIFO
#endif

/* Regular file bit */
#if !defined(S_IFREG)
#   define S_IFREG _S_IFREG
#endif

/* Read permission */
#if !defined(S_IREAD)
#   define S_IREAD _S_IREAD
#endif

/* Write permission */
#if !defined(S_IWRITE)
#   define S_IWRITE _S_IWRITE
#endif

/* Execute permission */
#if !defined(S_IEXEC)
#   define S_IEXEC _S_IEXEC
#endif

/* Pipe */
#if !defined(S_IFIFO)
#   define S_IFIFO _S_IFIFO
#endif

/* Block device */
#if !defined(S_IFBLK)
#   define S_IFBLK 0
#endif

/* Link */
#if !defined(S_IFLNK)
#   define S_IFLNK 0
#endif

/* Socket */
#if !defined(S_IFSOCK)
#   define S_IFSOCK 0
#endif

/* Read user permission */
#if !defined(S_IRUSR)
#   define S_IRUSR S_IREAD
#endif

/* Write user permission */
#if !defined(S_IWUSR)
#   define S_IWUSR S_IWRITE
#endif

/* Execute user permission */
#if !defined(S_IXUSR)
#   define S_IXUSR 0
#endif

/* Read group permission */
#if !defined(S_IRGRP)
#   define S_IRGRP 0
#endif

/* Write group permission */
#if !defined(S_IWGRP)
#   define S_IWGRP 0
#endif

/* Execute group permission */
#if !defined(S_IXGRP)
#   define S_IXGRP 0
#endif

/* Read others permission */
#if !defined(S_IROTH)
#   define S_IROTH 0
#endif

/* Write others permission */
#if !defined(S_IWOTH)
#   define S_IWOTH 0
#endif

/* Execute others permission */
#if !defined(S_IXOTH)
#   define S_IXOTH 0
#endif

/* Maximum length of file name */
#if !defined(PATH_MAX)
#   define PATH_MAX MAX_PATH
#endif
#if !defined(FILENAME_MAX)
#   define FILENAME_MAX MAX_PATH
#endif
#if !defined(NAME_MAX)
#   define NAME_MAX FILENAME_MAX
#endif

/* File type flags for d_type */
#define DT_UNKNOWN 0
#define DT_REG S_IFREG
#define DT_DIR S_IFDIR
#define DT_FIFO S_IFIFO
#define DT_SOCK S_IFSOCK
#define DT_CHR S_IFCHR
#define DT_BLK S_IFBLK
#define DT_LNK S_IFLNK

/* Macros for converting between st_mode and d_type */
#define IFTODT(mode) ((mode) & S_IFMT)
#define DTTOIF(type) (type)

/*
* File type macros.  Note that block devices, sockets and links cannot be
* distinguished on Windows and the macros S_ISBLK, S_ISSOCK and S_ISLNK are
* only defined for compatibility.  These macros should always return false
* on Windows.
*/
#if !defined(S_ISFIFO)
#   define S_ISFIFO(mode) (((mode) & S_IFMT) == S_IFIFO)
#endif
#if !defined(S_ISDIR)
#   define S_ISDIR(mode) (((mode) & S_IFMT) == S_IFDIR)
#endif
#if !defined(S_ISREG)
#   define S_ISREG(mode) (((mode) & S_IFMT) == S_IFREG)
#endif
#if !defined(S_ISLNK)
#   define S_ISLNK(mode) (((mode) & S_IFMT) == S_IFLNK)
#endif
#if !defined(S_ISSOCK)
#   define S_ISSOCK(mode) (((mode) & S_IFMT) == S_IFSOCK)
#endif
#if !defined(S_ISCHR)
#   define S_ISCHR(mode) (((mode) & S_IFMT) == S_IFCHR)
#endif
#if !defined(S_ISBLK)
#   define S_ISBLK(mode) (((mode) & S_IFMT) == S_IFBLK)
#endif

/* Return the exact length of d_namlen without zero terminator */
#define _D_EXACT_NAMLEN(p) ((p)->d_namlen)

/* Return number of bytes needed to store d_namlen */
#define _D_ALLOC_NAMLEN(p) (PATH_MAX)


#ifdef __cplusplus
extern "C" {
#endif


    /* Wide-character version */
    struct _wdirent {
        /* Always zero */
        long d_ino;

        /* Structure size */
        unsigned short d_reclen;

        /* Length of name without \0 */
        size_t d_namlen;

        /* File type */
        int d_type;

        /* File name */
        wchar_t d_name[PATH_MAX];
    };
    typedef struct _wdirent _wdirent;

    struct _WDIR {
        /* Current directory entry */
        struct _wdirent ent;

        /* Private file data */
        WIN32_FIND_DATAW data;

        /* True if data is valid */
        int cached;

        /* Win32 search handle */
        HANDLE handle;

        /* Initial directory name */
        wchar_t *patt;
    };
    typedef struct _WDIR _WDIR;

    static _WDIR *_wopendir(const wchar_t *dirname);
    static struct _wdirent *_wreaddir(_WDIR *dirp);
    static int _wclosedir(_WDIR *dirp);
    static void _wrewinddir(_WDIR* dirp);


    /* For compatibility with Symbian */
#define wdirent _wdirent
#define WDIR _WDIR
#define wopendir _wopendir
#define wreaddir _wreaddir
#define wclosedir _wclosedir
#define wrewinddir _wrewinddir


    /* Multi-byte character versions */
    struct dirent {
        /* Always zero */
        long d_ino;

        /* Structure size */
        unsigned short d_reclen;

        /* Length of name without \0 */
        size_t d_namlen;

        /* File type */
        int d_type;

        /* File name */
        char d_name[PATH_MAX];
    };
    typedef struct dirent dirent;

    struct DIR {
        struct dirent ent;
        struct _WDIR *wdirp;
    };
    typedef struct DIR DIR;

    static DIR *opendir(const char *dirname);
    static struct dirent *readdir(DIR *dirp);
    static int closedir(DIR *dirp);
    static void rewinddir(DIR* dirp);


    /* Internal utility functions */
    static WIN32_FIND_DATAW *dirent_first(_WDIR *dirp);
    static WIN32_FIND_DATAW *dirent_next(_WDIR *dirp);

    static int dirent_mbstowcs_s(
        size_t *pReturnValue,
        wchar_t *wcstr,
        size_t sizeInWords,
        const char *mbstr,
        size_t count);

    static int dirent_wcstombs_s(
        size_t *pReturnValue,
        char *mbstr,
        size_t sizeInBytes,
        const wchar_t *wcstr,
        size_t count);

    static void dirent_set_errno(int error);

    /*
    * Open directory stream DIRNAME for read and return a pointer to the
    * internal working area that is used to retrieve individual directory
    * entries.
    */
    static _WDIR*
        _wopendir(
            const wchar_t *dirname)
    {
        _WDIR *dirp = NULL;
        int error;

        /* Must have directory name */
        if (dirname == NULL || dirname[0] == '\0') {
            dirent_set_errno(ENOENT);
            return NULL;
        }

        /* Allocate new _WDIR structure */
        dirp = (_WDIR*)malloc(sizeof(struct _WDIR));
        if (dirp != NULL) {
            DWORD n;

            /* Reset _WDIR structure */
            dirp->handle = INVALID_HANDLE_VALUE;
            dirp->patt = NULL;
            dirp->cached = 0;

            /* Compute the length of full path plus zero terminator */
            n = GetFullPathNameW(dirname, 0, NULL, NULL);

            /* Allocate room for absolute directory name and search pattern */
            dirp->patt = (wchar_t*)malloc(sizeof(wchar_t) * n + 16);
            if (dirp->patt) {

                /*
                * Convert relative directory name to an absolute one.  This
                * allows rewinddir() to function correctly even when current
                * working directory is changed between opendir() and rewinddir().
                */
                n = GetFullPathNameW(dirname, n, dirp->patt, NULL);
                if (n > 0) {
                    wchar_t *p;

                    /* Append search pattern \* to the directory name */
                    p = dirp->patt + n;
                    if (dirp->patt < p) {
                        switch (p[-1]) {
                        case '\\':
                        case '/':
                        case ':':
                            /* Directory ends in path separator, e.g. c:\temp\ */
                            /*NOP*/;
                            break;

                        default:
                            /* Directory name doesn't end in path separator */
                            *p++ = '\\';
                        }
                    }
                    *p++ = '*';
                    *p = '\0';

                    /* Open directory stream and retrieve the first entry */
                    if (dirent_first(dirp)) {
                        /* Directory stream opened successfully */
                        error = 0;
                    }
                    else {
                        /* Cannot retrieve first entry */
                        error = 1;
                        dirent_set_errno(ENOENT);
                    }

                }
                else {
                    /* Cannot retrieve full path name */
                    dirent_set_errno(ENOENT);
                    error = 1;
                }

            }
            else {
                /* Cannot allocate memory for search pattern */
                error = 1;
            }

        }
        else {
            /* Cannot allocate _WDIR structure */
            error = 1;
        }

        /* Clean up in case of error */
        if (error  &&  dirp) {
            _wclosedir(dirp);
            dirp = NULL;
        }

        return dirp;
    }

    /*
    * Read next directory entry.  The directory entry is returned in dirent
    * structure in the d_name field.  Individual directory entries returned by
    * this function include regular files, sub-directories, pseudo-directories
    * "." and ".." as well as volume labels, hidden files and system files.
    */
    static struct _wdirent*
        _wreaddir(
            _WDIR *dirp)
    {
        WIN32_FIND_DATAW *datap;
        struct _wdirent *entp;

        /* Read next directory entry */
        datap = dirent_next(dirp);
        if (datap) {
            size_t n;
            DWORD attr;

            /* Pointer to directory entry to return */
            entp = &dirp->ent;

            /*
            * Copy file name as wide-character string.  If the file name is too
            * long to fit in to the destination buffer, then truncate file name
            * to PATH_MAX characters and zero-terminate the buffer.
            */
            n = 0;
            while (n + 1 < PATH_MAX  &&  datap->cFileName[n] != 0) {
                entp->d_name[n] = datap->cFileName[n];
                n++;
            }
            dirp->ent.d_name[n] = 0;

            /* Length of file name excluding zero terminator */
            entp->d_namlen = n;

            /* File type */
            attr = datap->dwFileAttributes;
            if ((attr & FILE_ATTRIBUTE_DEVICE) != 0) {
                entp->d_type = DT_CHR;
            }
            else if ((attr & FILE_ATTRIBUTE_DIRECTORY) != 0) {
                entp->d_type = DT_DIR;
            }
            else {
                entp->d_type = DT_REG;
            }

            /* Reset dummy fields */
            entp->d_ino = 0;
            entp->d_reclen = sizeof(struct _wdirent);

        }
        else {

            /* Last directory entry read */
            entp = NULL;

        }

        return entp;
    }

    /*
    * Close directory stream opened by opendir() function.  This invalidates the
    * DIR structure as well as any directory entry read previously by
    * _wreaddir().
    */
    static int
        _wclosedir(
            _WDIR *dirp)
    {
        int ok;
        if (dirp) {

            /* Release search handle */
            if (dirp->handle != INVALID_HANDLE_VALUE) {
                FindClose(dirp->handle);
                dirp->handle = INVALID_HANDLE_VALUE;
            }

            /* Release search pattern */
            if (dirp->patt) {
                free(dirp->patt);
                dirp->patt = NULL;
            }

            /* Release directory structure */
            free(dirp);
            ok = /*success*/0;

        }
        else {
            /* Invalid directory stream */
            dirent_set_errno(EBADF);
            ok = /*failure*/-1;
        }
        return ok;
    }

    /*
    * Rewind directory stream such that _wreaddir() returns the very first
    * file name again.
    */
    static void
        _wrewinddir(
            _WDIR* dirp)
    {
        if (dirp) {
            /* Release existing search handle */
            if (dirp->handle != INVALID_HANDLE_VALUE) {
                FindClose(dirp->handle);
            }

            /* Open new search handle */
            dirent_first(dirp);
        }
    }

    /* Get first directory entry (internal) */
    static WIN32_FIND_DATAW*
        dirent_first(
            _WDIR *dirp)
    {
        WIN32_FIND_DATAW *datap;

        /* Open directory and retrieve the first entry */
        dirp->handle = FindFirstFileW(dirp->patt, &dirp->data);
        if (dirp->handle != INVALID_HANDLE_VALUE) {

            /* a directory entry is now waiting in memory */
            datap = &dirp->data;
            dirp->cached = 1;

        }
        else {

            /* Failed to re-open directory: no directory entry in memory */
            dirp->cached = 0;
            datap = NULL;

        }
        return datap;
    }

    /* Get next directory entry (internal) */
    static WIN32_FIND_DATAW*
        dirent_next(
            _WDIR *dirp)
    {
        WIN32_FIND_DATAW *p;

        /* Get next directory entry */
        if (dirp->cached != 0) {

            /* A valid directory entry already in memory */
            p = &dirp->data;
            dirp->cached = 0;

        }
        else if (dirp->handle != INVALID_HANDLE_VALUE) {

            /* Get the next directory entry from stream */
            if (FindNextFileW(dirp->handle, &dirp->data) != FALSE) {
                /* Got a file */
                p = &dirp->data;
            }
            else {
                /* The very last entry has been processed or an error occured */
                FindClose(dirp->handle);
                dirp->handle = INVALID_HANDLE_VALUE;
                p = NULL;
            }

        }
        else {

            /* End of directory stream reached */
            p = NULL;

        }

        return p;
    }

    /*
    * Open directory stream using plain old C-string.
    */
    static DIR*
        opendir(
            const char *dirname)
    {
        struct DIR *dirp;
        int error;

        /* Must have directory name */
        if (dirname == NULL || dirname[0] == '\0') {
            dirent_set_errno(ENOENT);
            return NULL;
        }

        /* Allocate memory for DIR structure */
        dirp = (DIR*)malloc(sizeof(struct DIR));
        if (dirp) {
            wchar_t wname[PATH_MAX];
            size_t n;

            /* Convert directory name to wide-character string */
            error = dirent_mbstowcs_s(&n, wname, PATH_MAX, dirname, PATH_MAX);
            if (!error) {

                /* Open directory stream using wide-character name */
                dirp->wdirp = _wopendir(wname);
                if (dirp->wdirp) {
                    /* Directory stream opened */
                    error = 0;
                }
                else {
                    /* Failed to open directory stream */
                    error = 1;
                }

            }
            else {
                /*
                * Cannot convert file name to wide-character string.  This
                * occurs if the string contains invalid multi-byte sequences or
                * the output buffer is too small to contain the resulting
                * string.
                */
                error = 1;
            }

        }
        else {
            /* Cannot allocate DIR structure */
            error = 1;
        }

        /* Clean up in case of error */
        if (error  &&  dirp) {
            free(dirp);
            dirp = NULL;
        }

        return dirp;
    }

    /*
    * Read next directory entry.
    *
    * When working with text consoles, please note that file names returned by
    * readdir() are represented in the default ANSI code page while any output to
    * console is typically formatted on another code page.  Thus, non-ASCII
    * characters in file names will not usually display correctly on console.  The
    * problem can be fixed in two ways: (1) change the character set of console
    * to 1252 using chcp utility and use Lucida Console font, or (2) use
    * _cprintf function when writing to console.  The _cprinf() will re-encode
    * ANSI strings to the console code page so many non-ASCII characters will
    * display correcly.
    */
    static struct dirent*
        readdir(
            DIR *dirp)
    {
        WIN32_FIND_DATAW *datap;
        struct dirent *entp;

        /* Read next directory entry */
        datap = dirent_next(dirp->wdirp);
        if (datap) {
            size_t n;
            int error;

            /* Attempt to convert file name to multi-byte string */
            error = dirent_wcstombs_s(
                &n, dirp->ent.d_name, PATH_MAX, datap->cFileName, PATH_MAX);

            /*
            * If the file name cannot be represented by a multi-byte string,
            * then attempt to use old 8+3 file name.  This allows traditional
            * Unix-code to access some file names despite of unicode
            * characters, although file names may seem unfamiliar to the user.
            *
            * Be ware that the code below cannot come up with a short file
            * name unless the file system provides one.  At least
            * VirtualBox shared folders fail to do this.
            */
            if (error  &&  datap->cAlternateFileName[0] != '\0') {
                error = dirent_wcstombs_s(
                    &n, dirp->ent.d_name, PATH_MAX,
                    datap->cAlternateFileName, PATH_MAX);
            }

            if (!error) {
                DWORD attr;

                /* Initialize directory entry for return */
                entp = &dirp->ent;

                /* Length of file name excluding zero terminator */
                entp->d_namlen = n - 1;

                /* File attributes */
                attr = datap->dwFileAttributes;
                if ((attr & FILE_ATTRIBUTE_DEVICE) != 0) {
                    entp->d_type = DT_CHR;
                }
                else if ((attr & FILE_ATTRIBUTE_DIRECTORY) != 0) {
                    entp->d_type = DT_DIR;
                }
                else {
                    entp->d_type = DT_REG;
                }

                /* Reset dummy fields */
                entp->d_ino = 0;
                entp->d_reclen = sizeof(struct dirent);

            }
            else {
                /*
                * Cannot convert file name to multi-byte string so construct
                * an errornous directory entry and return that.  Note that
                * we cannot return NULL as that would stop the processing
                * of directory entries completely.
                */
                entp = &dirp->ent;
                entp->d_name[0] = '?';
                entp->d_name[1] = '\0';
                entp->d_namlen = 1;
                entp->d_type = DT_UNKNOWN;
                entp->d_ino = 0;
                entp->d_reclen = 0;
            }

        }
        else {
            /* No more directory entries */
            entp = NULL;
        }

        return entp;
    }

    /*
    * Close directory stream.
    */
    static int
        closedir(
            DIR *dirp)
    {
        int ok;
        if (dirp) {

            /* Close wide-character directory stream */
            ok = _wclosedir(dirp->wdirp);
            dirp->wdirp = NULL;

            /* Release multi-byte character version */
            free(dirp);

        }
        else {

            /* Invalid directory stream */
            dirent_set_errno(EBADF);
            ok = /*failure*/-1;

        }
        return ok;
    }

    /*
    * Rewind directory stream to beginning.
    */
    static void
        rewinddir(
            DIR* dirp)
    {
        /* Rewind wide-character string directory stream */
        _wrewinddir(dirp->wdirp);
    }

    /* Convert multi-byte string to wide character string */
    static int
        dirent_mbstowcs_s(
            size_t *pReturnValue,
            wchar_t *wcstr,
            size_t sizeInWords,
            const char *mbstr,
            size_t count)
    {
        int error;

#if defined(_MSC_VER)  &&  _MSC_VER >= 1400

        /* Microsoft Visual Studio 2005 or later */
        error = mbstowcs_s(pReturnValue, wcstr, sizeInWords, mbstr, count);

#else

        /* Older Visual Studio or non-Microsoft compiler */
        size_t n;

        /* Convert to wide-character string (or count characters) */
        n = mbstowcs(wcstr, mbstr, sizeInWords);
        if (!wcstr || n < count) {

            /* Zero-terminate output buffer */
            if (wcstr  &&  sizeInWords) {
                if (n >= sizeInWords) {
                    n = sizeInWords - 1;
                }
                wcstr[n] = 0;
            }

            /* Length of resuting multi-byte string WITH zero terminator */
            if (pReturnValue) {
                *pReturnValue = n + 1;
            }

            /* Success */
            error = 0;

        }
        else {

            /* Could not convert string */
            error = 1;

        }

#endif

        return error;
    }

    /* Convert wide-character string to multi-byte string */
    static int
        dirent_wcstombs_s(
            size_t *pReturnValue,
            char *mbstr,
            size_t sizeInBytes, /* max size of mbstr */
            const wchar_t *wcstr,
            size_t count)
    {
        int error;

#if defined(_MSC_VER)  &&  _MSC_VER >= 1400

        /* Microsoft Visual Studio 2005 or later */
        error = wcstombs_s(pReturnValue, mbstr, sizeInBytes, wcstr, count);

#else

        /* Older Visual Studio or non-Microsoft compiler */
        size_t n;

        /* Convert to multi-byte string (or count the number of bytes needed) */
        n = wcstombs(mbstr, wcstr, sizeInBytes);
        if (!mbstr || n < count) {

            /* Zero-terminate output buffer */
            if (mbstr  &&  sizeInBytes) {
                if (n >= sizeInBytes) {
                    n = sizeInBytes - 1;
                }
                mbstr[n] = '\0';
            }

            /* Lenght of resulting multi-bytes string WITH zero-terminator */
            if (pReturnValue) {
                *pReturnValue = n + 1;
            }

            /* Success */
            error = 0;

        }
        else {

            /* Cannot convert string */
            error = 1;

        }

#endif

        return error;
    }

    /* Set errno variable */
    static void
        dirent_set_errno(
            int error)
    {
#if defined(_MSC_VER)  &&  _MSC_VER >= 1400

        /* Microsoft Visual Studio 2005 and later */
        _set_errno(error);

#else

        /* Non-Microsoft compiler or older Microsoft compiler */
        errno = error;

#endif
    }


#ifdef __cplusplus
}
#endif
#include <io.h>
#else
#include <dirent.h>
// #include </usr/include/x86_64-linux-gnu/sys/io.h>
#endif

#ifdef USE_OMP
#include <omp.h>
#else
int omp_get_max_threads();
int omp_get_thread_num();
// int omp_set_num_threads(int);
#endif

typedef std::vector<cv::Point> VP;
typedef std::vector< VP >      VVP;
typedef unsigned int uint;


void _list_dir(std::string dir, std::vector<std::string>& files, std::string suffixs = "", bool r = false);

std::vector<std::string> _split(const std::string& srcstr, const std::string& delimeter);
bool _startswith(const std::string& str, const std::string& start);
bool _endswith(const std::string& str, const std::string& end);
void _randperm(int n, int m, int arr[], bool sort_ = true);

/***************** math-related functions ****************/
float _atan2(float y, float x);
void _mean_std(std::vector<float>& vec, float& mean, float& std);
int inline _sgn(float val) { return (0.f < val) - (val < 0.f); }
float inline _ed2(const cv::Point& A, const cv::Point& B)
{
    return float(((B.x - A.x)*(B.x - A.x) + (B.y - A.y)*(B.y - A.y)));
}
float _get_min_angle_PI(float alpha, float beta);

double inline _tic()
{
    return (double)cv::getTickCount();
}
double inline _toc(double tic) // ms
{
    return ((double)cv::getTickCount() - tic)*1000. / cv::getTickFrequency();
}
inline int _isnan(double x) { return x != x; }


void _tag_canny(cv::InputArray image, cv::OutputArray _edges,
    cv::OutputArray _sobel_x, cv::OutputArray _sobel_y,
    int apertureSize, bool L2gradient, double percent_ne);


void _find_contours_oneway(cv::Mat1b& image, VVP& segments, int iMinLength);
void _find_contours_eight(cv::Mat1b& image, std::vector<VVP>& segments, int iMinLength);
void _show_contours_eight(cv::Mat1b& image, std::vector<VVP>& segments, const char* title);

void _tag_find_contours(cv::Mat1b& image, VVP& segments, int iMinLength);
void _tag_show_contours(cv::Mat1b& image, VVP& segments, const char* title);
void _tag_show_contours(cv::Size& imsz, VVP& segments, const char* title);

bool _SortBottomLeft2TopRight(const cv::Point& lhs, const cv::Point& rhs);
bool _SortBottomLeft2TopRight2f(const cv::Point2f& lhs, const cv::Point2f& rhs);
bool _SortTopLeft2BottomRight(const cv::Point& lhs, const cv::Point& rhs);


#ifndef M_PI
#define M_PI   3.14159265358979323846
#endif
#define M_2__PI        6.28318530718
#define M_1_2_PI       1.57079632679

// Elliptical struct definition
class Ellipse
{
public:
    float xc_;
    float yc_;
    float a_;
    float b_;
    float rad_;
    float score_;

    // Elliptic General equations Ax^2 + Bxy + Cy^2 + Dx + Ey + 1 = 0
    float A_;
    float B_;
    float C_;
    float D_;
    float E_;
    float F_;

    Ellipse() : xc_(0.f), yc_(0.f), a_(0.f), b_(0.f), rad_(0.f), score_(0.f),
        A_(0.f), B_(0.f), C_(0.f), D_(0.f), E_(0.f), F_(1.f) {}
    Ellipse(float xc, float yc, float a, float b, float rad, float score = 0.f) : xc_(xc), yc_(yc), a_(a), b_(b), rad_(rad), score_(score) {}
    Ellipse(const Ellipse& other) : xc_(other.xc_), yc_(other.yc_), a_(other.a_), b_(other.b_), rad_(other.rad_), score_(other.score_),
        A_(other.A_), B_(other.B_), C_(other.C_), D_(other.D_), E_(other.E_) {}

    void Draw(cv::Mat& img, const cv::Scalar& color, const int thickness)
    {
        if (IsValid())
            ellipse(img, cv::Point(cvRound(xc_), cvRound(yc_)), cv::Size(cvRound(a_), cvRound(b_)), rad_ * 180.0 / CV_PI, 0.0, 360.0, color, thickness);
    }

    void Draw(cv::Mat3b& img, const int thickness)
    {
        cv::Scalar color(0, cvFloor(255.f * score_), 0);
        if (IsValid())
            ellipse(img, cv::Point(cvRound(xc_), cvRound(yc_)), cv::Size(cvRound(a_), cvRound(b_)), rad_ * 180.0 / CV_PI, 0.0, 360.0, color, thickness);
    }

    bool operator<(const Ellipse& other) const
    {	// use for sorting
        if (score_ == other.score_)
        {
            float lhs_e = b_ / a_;
            float rhs_e = other.b_ / other.a_;
            if (lhs_e == rhs_e)
            {
                return false;
            }
            return lhs_e > rhs_e;
        }
        return score_ > other.score_;
    }

    // Elliptic General equations Ax^2 + Bxy + Cy^2 + Dx + Ey + F = 0
    void TransferFromGeneral() {
        float denominator = (B_*B_ - 4 * A_*C_);

        xc_ = (2 * C_*D_ - B_*E_) / denominator;
        yc_ = (2 * A_*E_ - B_*D_) / denominator;

        float pre = 2 * (A_*E_*E_ + C_*D_*D_ - B_*D_*E_ + denominator*F_);
        float lst = sqrt((A_ - C_)*(A_ - C_) + B_*B_);

        a_ = -sqrt(pre*(A_ + C_ + lst)) / denominator;
        b_ = -sqrt(pre*(A_ + C_ - lst)) / denominator;

        if (B_ == 0 && A_<C_)
            rad_ = 0;
        else if (B_ == 0 && A_>C_)
            rad_ = CV_PI / 2;
        else
            rad_ = atan((C_ - A_ - lst) / B_);
    }

    // Elliptic General equations Ax^2 + Bxy + Cy^2 + Dx + Ey + F = 0
    void TransferToGeneral() {
        A_ = a_*a_*sin(rad_)*sin(rad_) + b_*b_*cos(rad_)*cos(rad_);
        B_ = 2.f*(b_*b_ - a_*a_)*sin(rad_)*cos(rad_);
        C_ = a_*a_*cos(rad_)*cos(rad_) + b_*b_*sin(rad_)*sin(rad_);
        D_ = -2.f*A_*xc_ - B_*yc_;
        E_ = -B_*xc_ - 2.f*C_*yc_;
        F_ = A_*xc_*xc_ + B_*xc_*yc_ + C_*yc_*yc_ - a_*a_*b_*b_;
    }

    void GetRectangle(cv::Rect& rect) {
        float sin_theta = sin(-rad_);
        float cos_theta = cos(-rad_);
        float A = a_*a_ * sin_theta * sin_theta + b_* b_ * cos_theta * cos_theta;
        float B = 2 * (a_* a_ - b_ * b_) * sin_theta * cos_theta;
        float C = a_* a_ * cos_theta * cos_theta + b_ * b_ * sin_theta * sin_theta;
        float F = - a_ * a_ * b_ * b_;

        float y = sqrt(4 * A * F / (B * B - 4 * A * C));
        float y1 = -abs(y), y2 = abs(y);
        float x = sqrt(4 * C * F / (B * B - 4 * C * A));
        float x1 = -abs(x), x2 = abs(x);
    
        rect = cv::Rect(int(round(xc_ + x1)), int(round(yc_ + y1)), int(round(x2 - x1)), int(round(y2 - y1)));
    }

    float Perimeter() {
        // return 2*CV_PI*b_ + 4*(a_ - b_);
        return CV_PI*(3.f*(a_ + b_) - sqrt((3.f*a_ + b_)*(a_ + 3.f*b_)));
    }

    float Area() {
        return CV_PI*a_*b_;
    }

    bool IsValid() {
        bool nan = isnan(xc_) || isnan(yc_) || isnan(a_) || isnan(b_) || isnan(rad_);
        return !nan;
    }
};

// Data available after selection strategy.
// They are kept in an associative array to:
// 1) avoid recomputing data when starting from same arcs
// 2) be reused in firther proprecessing
struct EllipseData
{
    bool isValid;
    float ta;               // arc_a center line gradient
    float tb;               // arc_b
    float ra;               // gradient of a (slope of start of chord_1 and center of chord_2)
    float rb;               // gradient of b (slope of center of chord_1 and last of chord_2)
    cv::Point2f Ma;         // arc_a center of element
    cv::Point2f Mb;         // arc_b
    cv::Point2f Cab;        // center of ellipse
    std::vector<float> Sa;  // arc_a's center line of parallel chords
    std::vector<float> Sb;  // arc_b's center line of parallel chords
};

struct EllipseThreePoint
{
    bool isValid;
    cv::Point Cab;
    VP ArcI;
    VP ArcJ;
    VP ArcK;
};

/********************** EllipseFitting functions **********************/
void _ellipse_foci(float *param, float *foci);
float _ellipse_normal_angle(float x, float y, float *foci);
float _angle_diff(float a, float b);

/*************************** CNC functions ****************************/
float _value4SixPoints(cv::Point2f p3, cv::Point2f p2, cv::Point2f p1, cv::Point2f p4, cv::Point2f p5, cv::Point2f p6);


/**************** ellipse-evaluation-related functions ****************/
void _load_ellipse_GT(const std::string& gt_file_name, std::vector<Ellipse> & gt_ellipses, bool is_angle_radians = true);
void _load_ellipse_DT(const std::string& dt_file_name, std::vector<Ellipse> & dt_ellipses, bool is_angle_radians = true);

bool _ellipse_overlap(const cv::Mat1b& gt, const cv::Mat1b& dt, float th);
float _ellipse_overlap_real(const cv::Mat1b& gt, const cv::Mat1b& dt);
int _bool_count(const std::vector<bool> vb);
float _ellipse_evaluate_one(const std::vector<Ellipse>& ell_gt, const std::vector<Ellipse>& ell_dt, const cv::Mat3b& img);
float _ellipse_evaluate(std::vector<std::string>& image_fns, std::vector<std::string>& gt_fns, std::vector<std::string>& dt_fns,
    bool gt_angle_radians = true);




class EllipseDetector
{
    // Parameters

    // Preprocessing - Gaussian filter. See Sect [] in the paper
    cv::Size	szPreProcessingGaussKernel_;	    // size of the Gaussian filter in preprocessing step
    double	dPreProcessingGaussSigma_;			// sigma of the Gaussian filter in the preprocessing step


                                                // Selection strategy - Step 1 - Discard noisy or straight arcs. See Sect [] in the paper
    int		iMinEdgeLength_;					// minimum edge size
    float	fMinOrientedRectSide_;				// minumum size of the oriented bounding box containing the arc
    float	fMaxRectAxesRatio_;					// maximum aspect ratio of the oriented bounding box containing the arc

                                                // Selection strategy - Step 2 - Remove according to mutual convexities. See Sect [] in the paper
    float   fThrArcPosition_;

    // Selection Strategy - Step 3 - Number of points considered for slope estimation when estimating the center. See Sect [] in the paper
    unsigned uNs_;                              // Find at most Ns parallel chords.

                                                // Selection strategy - Step 3 - Discard pairs of arcs if their estimated center is not close enough. See Sect [] in the paper
    float	fMaxCenterDistance_;				// maximum distance in pixel between 2 center points
    float	fMaxCenterDistance2_;				// _fMaxCenterDistance * _fMaxCenterDistance

                                                // Validation - Points within a this threshold are considered to lie on the ellipse contour. See Sect [] in the paper
    float	fDistanceToEllipseContour_;			// maximum distance between a point and the contour. See equation [] in the paper

                                                // Validation - Assign a score. See Sect [] in the paper
    float	fMinScore_;							// minimum score to confirm a detection
    float	fMinReliability_;					// minimum auxiliary score to confirm a detection

    double  dPercentNe_;

    float fT_CNC_;
    float fT_TCN_L_; // filter lines
    float fT_TCN_P_;
    float fThre_r_;

                                                // auxiliary variables
    cv::Size	szIm_;			// input image size

    std::vector<double> times_;	// times_ is a vector containing the execution time of each step.

    int ACC_N_SIZE;			// size of accumulator N = B/A
    int ACC_R_SIZE;			// size of accumulator R = rho = atan(K)
    int ACC_A_SIZE;			// size of accumulator A

    int* accN;				// pointer to accumulator N
    int* accR;				// pointer to accumulator R
    int* accA;				// pointer to accumulator A

    cv::Mat1f EO_;

    VVP points_1, points_2, points_3, points_4;		// vector of points, one for each convexity class

public:

    // Constructor and Destructor
    EllipseDetector(void);
    ~EllipseDetector(void);

    // Detect the ellipses in the gray image
    void Detect(cv::Mat3b& I, std::vector<Ellipse>& ellipses);
    void Detect(cv::Mat& I, std::vector<Ellipse>& ellipses);

    // Draw the first iTopN ellipses on output
    void DrawDetectedEllipses(cv::Mat& output, std::vector<Ellipse>& ellipses, int iTopN = 0, int thickness = 2);

    // Set the parameters of the detector
    void SetParameters(cv::Size	szPreProcessingGaussKernelSize,
        double	dPreProcessingGaussSigma,
        float 	fThPosition,
        float	fMaxCenterDistance,
        int		iMinEdgeLength,
        float	fMinOrientedRectSide,
        float	fDistanceToEllipseContour,
        float	fMinScore,
        float	fMinReliability,
        int     iNs,
        double  dPercentNe,
        float   fT_CNC,
        float   fT_TCN_L,
        float   fT_TCN_P,
        float   fThre_r
    );

    // Return the execution time
    double GetExecTime() {
        double time_all(0);
        for (size_t i = 0; i < times_.size(); i++) time_all += times_[i];
        return time_all;
    }
    std::vector<double> GetTimes() { return times_; }

    float countOfFindEllipse_;
    float countOfGetFastCenter_;

private:

    // keys for hash table
    static const ushort PAIR_12 = 0x00;
    static const ushort PAIR_23 = 0x01;
    static const ushort PAIR_34 = 0x02;
    static const ushort PAIR_14 = 0x03;

    // generate keys from pair and indicse
    uint inline GenerateKey(uchar pair, ushort u, ushort v);

    void PreProcessing(cv::Mat1b& I, cv::Mat1b& arcs8);
    void RemoveStraightLine(VVP& segments, VVP& segments_update, int id = 0);
    void PreProcessing(cv::Mat1b& I, cv::Mat1b& DP, cv::Mat1b& DN);

    void ClusterEllipses(std::vector<Ellipse>& ellipses);

    // int FindMaxK(const std::vector<int>& v) const;
    // int FindMaxN(const std::vector<int>& v) const;
    // int FindMaxA(const std::vector<int>& v) const;

    int FindMaxK(const int* v) const;
    int FindMaxN(const int* v) const;
    int FindMaxA(const int* v) const;

    float GetMedianSlope(std::vector<cv::Point2f>& med, cv::Point2f& M, std::vector<float>& slopes);
    void GetFastCenter(std::vector<cv::Point>& e1, std::vector<cv::Point>& e2, EllipseData& data);
    float GetMinAnglePI(float alpha, float beta);

    void DetectEdges13(cv::Mat1b& DP, VVP& points_1, VVP& points_3);
    void DetectEdges24(cv::Mat1b& DN, VVP& points_2, VVP& points_4);

    void ArcsCheck1234(VVP& points_1, VVP& points_2, VVP& points_3, VVP& points_4);

    void FindEllipses(cv::Point2f& center,
        VP& edge_i,
        VP& edge_j,
        VP& edge_k,
        EllipseData& data_ij,
        EllipseData& data_ik,
        Ellipse& ell
    );

    cv::Point2f GetCenterCoordinates(EllipseData& data_ij, EllipseData& data_ik);

    void Triplets124(VVP& pi,
        VVP& pj,
        VVP& pk,
        std::unordered_map<uint, EllipseData>& data,
        std::vector<Ellipse>& ellipses
    );

    void Triplets231(VVP& pi,
        VVP& pj,
        VVP& pk,
        std::unordered_map<uint, EllipseData>& data,
        std::vector<Ellipse>& ellipses
    );

    void Triplets342(VVP& pi,
        VVP& pj,
        VVP& pk,
        std::unordered_map<uint, EllipseData>& data,
        std::vector<Ellipse>& ellipses
    );

    void Triplets413(VVP& pi,
        VVP& pj,
        VVP& pk,
        std::unordered_map<uint, EllipseData>& data,
        std::vector<Ellipse>& ellipses
    );

    void Tic(unsigned idx = 0) //start
    {
        while (idx >= timesSign_.size()) {
            timesSign_.push_back(0);
            times_.push_back(.0);
        }
        timesSign_[idx] = 0;
        timesSign_[idx]++;
        times_[idx] = (double)cv::getTickCount();
    }

    void Toc(unsigned idx = 0, std::string step = "") //stop
    {
        assert(timesSign_[idx] == 1);
        timesSign_[idx]++;
        times_[idx] = ((double)cv::getTickCount() - times_[idx])*1000. / cv::getTickFrequency();
        // #ifdef DEBUG_SPEED
        std::cout << "Cost time: " << times_[idx] << " ms [" << idx << "] - " << step << std::endl;
        if (idx == times_.size() - 1)
            std::cout << "Totally cost time: " << this->GetExecTime() << " ms" << std::endl;
        // #endif
    }

private:
    std::vector<int> timesSign_;
};



#endif // SPIRE_ELLIPSEDETECTOR_H
