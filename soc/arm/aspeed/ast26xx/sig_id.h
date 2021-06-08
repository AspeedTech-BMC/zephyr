#ifndef __SIG_ID_H__
#define __SIG_ID_H__

/* Pin ID */
enum {
	SIG_NONE = -1,
#define SIG_DEFINE(sig, ...) sig,
	#include "sig_def_list.h"
#undef SIG_DEFINE
	MAX_SIG_ID,
};

#endif /* #ifndef __SIG_ID_H__ */
