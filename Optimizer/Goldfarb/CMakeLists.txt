FILE(GLOB_RECURSE headers *.hh)
FILE(GLOB_RECURSE sources *.cc)
add_library (SJGoldfarb SHARED ${headers} ${sources})
