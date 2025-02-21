#pragma once

#ifndef PIMPL_H_TV7E3C9K
#define PIMPL_H_TV7E3C9K

//! \brief Declares a pimpl pointer.
#define RPCLIB_DECLARE_PIMPL()                                                \
    struct impl; std::unique_ptr<impl> pimpl;

#endif /* end of include guard: PIMPL_H_TV7E3C9K */
