$env:PATH = "C:\msys64\mingw64\bin;$env:PATH"
$env:TIME_LIMIT_MS = "3000"
cd c:\dev\opt\longest-path-optil

# Ensure output directories exist
New-Item -ItemType Directory -Force -Path "tests" | Out-Null
New-Item -ItemType Directory -Force -Path "results" | Out-Null

Write-Host "=== Compiling ==="
g++ -O2 -std=c++17 -o solution.exe solution.cpp 2>&1
g++ -O2 -std=c++17 -o gen.exe gen.cpp 2>&1
g++ -O2 -std=c++17 -o validate.exe validate.cpp 2>&1

$tests = @(
    @{name="small_random";   type="random"; N=20;    M=50;    seed=1},
    @{name="medium_random";  type="random"; N=1000;  M=5000;  seed=2},
    @{name="tree";           type="tree";   N=1000;  M=999;   seed=3},
    @{name="grid";           type="grid";   N=900;   M=0;     seed=4},
    @{name="large_random";   type="random"; N=10000; M=50000; seed=5},
    @{name="dense";          type="random"; N=500;   M=20000; seed=6},
    @{name="huge_sparse";    type="random"; N=50000; M=100000;seed=7}
)

foreach ($t in $tests) {
    Write-Host "`n=== Test: $($t.name) (N=$($t.N), M=$($t.M)) ==="
    
    $testFile = "tests\test_$($t.name).txt"
    $outFile = "results\out_$($t.name).txt"

    # Generate graph input into tests/
    cmd /c ".\gen.exe $($t.type) $($t.N) $($t.M) $($t.seed) > $testFile"
    
    # Run solution and write to results/
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    cmd /c ".\solution.exe < $testFile > $outFile"
    $sw.Stop()
    
    # Validate result
    .\validate.exe $testFile $outFile
    Write-Host "Time: $($sw.ElapsedMilliseconds)ms"
}
