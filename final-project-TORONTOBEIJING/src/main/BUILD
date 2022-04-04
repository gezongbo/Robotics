load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "main",
    srcs = ["main.cc"],
    deps = ["//src/lib:TrojanMap",
            "//src/lib:UI",
            "//src/lib:MapUI",
            "@ncurses//:main",
	     	"@opencv//:opencv",],
)
