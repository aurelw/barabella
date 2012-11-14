#define BARABELLA_VERSION 0.1

#define BB_DEBUG_LVL 4
#if BB_DEBUG_LVL > 0
    #define BB_INFO
#endif
#if BB_DEBUG_LVL > 1
    #define BB_LOG
#endif
#if BB_DEBUG_LVL > 2
    #define BB_VERBOSE
#endif
# if BB_DEBUG_LVL > 3
    #define BB_FLOOD
#endif
