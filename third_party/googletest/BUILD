licenses(['permissive'])

cc_library(
  name = 'gmock',
  srcs = glob(['gmock/src/*.cc'], ['gmock/src/gmock-all.cc']),
  hdrs = glob(['gmock/include/*.h']),
  includes = ['gmock/include', '.'],
  deps = ['gtest'],
  visibility = ['//visibility:public']
)

cc_library(
  name = 'gtest',
  srcs = glob([
    'gtest/src/*',
    'gtest/include/gtest/internal/**/*.h'
  ], ['gtest/src/gtest-all.cc']),
  hdrs = glob(['gtest/include/gtest/*.h']),
  includes = [
    'gtest/include',
    'gtest'
  ],
  linkopts = ['-lpthread'],
  visibility = ['//visibility:public']
)
