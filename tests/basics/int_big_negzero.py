# test cases where -0 can be generated

large = 1 << 100
large_plus_one = large + 1
zero = large - large
negzero_a = -zero
negzero_b = large - large_plus_one + 1
negzero_c = (-large) >> 200
args = (0, zero, negzero_a, negzero_b, negzero_c, 1, -1)
for lhs in args:
    for rhs in args:
        print("{} == {} = {}".format(lhs, rhs, lhs == rhs))
        print("{} + {} = {}".format(lhs, rhs, lhs + rhs))
        print("{} - {} = {}".format(lhs, rhs, lhs - rhs))
        print("{} * {} = {}".format(lhs, rhs, lhs * rhs))
        print("{} | {} = {}".format(lhs, rhs, lhs | rhs))
        print("{} & {} = {}".format(lhs, rhs, lhs & rhs))
        print("{} ^ {} = {}".format(lhs, rhs, lhs ^ rhs))
