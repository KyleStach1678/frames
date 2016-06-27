cc_library(
  name = 'frame',
  srcs = ['frame.cpp'],
  hdrs = ['frame.h'],
  deps = ['//third_party/eigen'],
  linkopts = ['-lm']
)

cc_test(
  name = 'frame_test',
  srcs = ['frame_test.cpp'],
  deps = ['//third_party/googletest:gtest', ':frame']
)
