#define PTT_KEY_DEBUG_CODE

#define PTT_KEY_DEVICE "PTT_KEY"
#ifdef PTT_KEY_DEBUG_CODE
#undef PTT_KEY_DEBUG
#define PTT_KEY_DEBUG(a,arg...) pr_err(PTT_KEY_DEVICE ": " a, ##arg)
#define PTT_KEY_FUNC()	pr_err(PTT_KEY_DEVICE ": %s line=%d\n", __func__, __LINE__)
#else
#define PTT_KEY_DEBUG(arg...)
#define PTT_KEY_FUNC()
#endif
