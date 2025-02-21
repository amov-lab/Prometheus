echo "Creating release for Windows"

$RELEASE_OUTPUT_DIR = "release"
$SRC = "src"
$BUILD_DIR = "build"
$CMAKE_FLAGS = "-DRPCLIB_BUILD_TESTS=OFF"

Remove-Item -Force -Recurse $RELEASE_OUTPUT_DIR

mkdir $RELEASE_OUTPUT_DIR
pushd $RELEASE_OUTPUT_DIR

git clone --depth=1 https://github.com/rpclib/rpclib.git $SRC

$archs = @("x86"; "x64")
$configs = @("Debug"; "Release")
$runtimes = @("Static"; "Dynamic")

$runtime_suffixes = @{
	"DynamicRelease"="MD";
	"DynamicDebug"="MDd";
	"StaticRelease"="MT";
	"StaticDebug"="MTd"
}

$arch_generators = @{
	"x86"="Visual Studio 14 2015";
	"x64"="Visual Studio 14 2015 Win64"
}

$extra_cmake_flags = @{
	"MD"="";
	"MDd"="";
	"MT"="-DRPCLIB_MSVC_STATIC_RUNTIME=ON";
	"MTd"="-DRPCLIB_MSVC_STATIC_RUNTIME=ON"
}

function build_config($arch, $runtime, $config) {
	echo "Building $arch | $runtime | $config..."
	$special_build_dir = "build_$arch$runtime$config".ToLower()
	echo "Build directory: $special_build_dir"
	mkdir $special_build_dir
	pushd $special_build_dir

	$suffix = $runtime_suffixes["$runtime$config"]
	$gen = $arch_generators[$arch]
	$flags = $extra_cmake_flags[$suffix]
	$name_suff = "{0}-{1}" -f $suffix.ToLower(),$arch
	$name_suff.replace(' ', '')

	cmake ../$SRC -G $gen $CMAKE_FLAGS $flags -DRPCLIB_NAME_SUFFIX="$name_suff"
	cmake --build . --config $config

	popd
}

function get_macro($file, $macro) {
	$m = $file | Select-String -Pattern "${macro} ([0-9]+)" -AllMatches
	return $m.Matches.Groups[1].Value
}

foreach ($arch in $archs) {
	foreach ($config in $configs) {
		foreach ($runtime in $runtimes) {
			build_config $arch $runtime $config
		}
	}
}

popd

$pkg = "pkg"
mkdir $pkg\lib
mkdir $pkg\include

Get-ChildItem .\$RELEASE_OUTPUT_DIR\* -Recurse -Include *.pdb, *.lib | Copy-Item -Destination $pkg\lib
Get-ChildItem -Path ..\include | Copy-Item -Destination .\$pkg\include\ -Recurse -Container

$verfile = Get-Content $pkg\include\rpc\version.h

$v_major = get_macro($verfile, "RPCLIB_VERSION_MAJOR").Trim()
$v_minor = get_macro($verfile, "RPCLIB_VERSION_MINOR").Trim()
$v_build = get_macro($verfile, "RPCLIB_VERSION_BUILD").Trim()

$zipname = "rpclib-msvc2015-{0}.{1}.{2}.zip" -f $v_major,$v_minor,$v_build
$zipname = $zipname.replace(' ', '')
Compress-Archive -Path $pkg\* -Destination $zipname -Force

Remove-Item -Force -Recurse $RELEASE_OUTPUT_DIR
Remove-Item -Force -Recurse $pkg


