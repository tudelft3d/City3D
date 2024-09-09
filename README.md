# 

Anlegen der venv:

```shell
python3 -m venv .venv
```

Starten der venv:

```shell
source .venv/bin/activate # Linux
source .venv/Scripts/activate # Windows
```

Einrichten der venv, indem alle Abhängkeiten aus der requirements.txt übernommen werden:

```shell
python3 -m pip install conan
```

conan Abhängigkeiten installieren und bauen:

```shell
conan install . --build=missing
cd build/Release
source ./generators/conanbuild.sh
cmake ../.. -DCMAKE_TOOLCHAIN_FILE=generators/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build .

./bin/city3d -h
```