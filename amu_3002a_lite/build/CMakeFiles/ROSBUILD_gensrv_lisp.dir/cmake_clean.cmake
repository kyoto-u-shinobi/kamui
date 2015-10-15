FILE(REMOVE_RECURSE
  "../srv_gen"
  "../srv_gen"
  "../src/amu_3002a_lite/srv"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/amu_control.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_amu_control.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
