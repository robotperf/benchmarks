# Copyright 2026 Daniil Mordanov
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ros2benchmark.verb import Benchmark


def test_missing_result_fields_do_not_abort_loading(tmp_path, capsys):
    yaml_file = tmp_path / "benchmark.yaml"
    yaml_file.write_text(
        """
id: test
name: Test benchmark
description: Test benchmark data
short: Test
graph: graph.svg
reproduction: ros2 benchmark test
results:
  - result:
      metric_unit: ms
      type: grey
      hardware: test-hardware
      category: edge
      value: 1.0
      note: missing metric and timestamp
      datasource: test
  - result:
      metric: latency
      metric_unit: ms
      type: grey
      hardware: test-hardware
      category: edge
      value: 1.0
      note: missing timestamp
      datasource: test
  - result:
      metric: latency
      metric_unit: ms
      type: grey
      hardware: test-hardware
      category: edge
      timestampt: 2026-08-10
      value: 1.0
      note: valid result
      datasource: test
""",
        encoding="utf-8",
    )

    benchmark = Benchmark(str(yaml_file))

    assert len(benchmark.results) == 1
    assert benchmark.results[0]["note"] == "valid result"
    output = capsys.readouterr().out
    assert "'metric' not found" in output
    assert "'timestampt' not found" in output
    assert output.count("timestamp: unknown") == 2
