# inputs
FEATURES_OPTIONAL :=
FEATURES_REQUIRED :=
FEATURES_REQUIRED_ANY := d|b|a
FEATURES_PROVIDED := c
FEATURES_BLACKLIST :=
FEATURES_CONFLICT :=

# expected results
EXPECTED_FEATURES_USED := d
EXPECTED_FEATURES_MISSING := d
EXPECTED_FEATURES_USED_BLACKLISTED :=
EXPECTED_FEATURES_CONFLICTING :=

include Makefile.test
