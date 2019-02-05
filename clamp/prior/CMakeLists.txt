# Install headers
file(GLOB prior_headers "*.h")
install(FILES ${prior_headers} DESTINATION include/clamp/prior)

# Build tests
gtsamAddTestsGlob(prior "tests/*.cpp" "" ${PROJECT_NAME})
