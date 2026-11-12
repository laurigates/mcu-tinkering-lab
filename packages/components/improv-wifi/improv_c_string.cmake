# improv_c_string_literal(<out_var> <text>)
#
# Sets <out_var> to <text> as a C string literal, quotes included, with every
# backslash and double quote escaped. The component's CMakeLists.txt uses it to
# pass the flasher.json `name` as IMPROV_FIRMWARE_NAME: pasted in unescaped, a
# name containing `"` or `\` breaks the compile of every project that builds
# the component. The robocar host suite (test/CMakeLists.txt,
# test_improv_firmware_name) compiles a definition made by this function from a
# name holding both.
function(improv_c_string_literal out_var text)
    string(REPLACE "\\" "\\\\" escaped "${text}")
    string(REPLACE "\"" "\\\"" escaped "${escaped}")
    set(${out_var} "\"${escaped}\"" PARENT_SCOPE)
endfunction()
