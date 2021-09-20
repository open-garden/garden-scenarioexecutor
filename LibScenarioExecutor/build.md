# ビルド手順

ビルドにはpicojsonとpybind11が必要です。

1. 以下のコマンドでpicojsonをクローンしてください。  
   ```
   cd ~/ScenarioExecutor/LibScenarioExecutor
   git clone https://github.com/kazuho/picojson
   ```

1. 以下のコマンドでpybind11をクローンしてください。  
   ```
   cd ~/ScenarioExecutor/LibScenarioExecutor
   git clone https://github.com/pybind/pybind11
   ```

1. 以下のコマンドでビルドしてください。
   ```
   cd ~/ScenarioExecutor/LibScenarioExecutor/build
   cmake ..
   make
   ```
