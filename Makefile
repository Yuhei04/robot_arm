# Usage:
#   make build
#   make upload
#   make monitor
#   make flash
#   make ports
#
# Override examples:
#   make upload PORT=/dev/cu.usbmodemXXXX
#   make monitor BAUD=115200

PROJ ?= sketches/opencr_dxl_check
FQBN ?= OpenCR:OpenCR:OpenCR
BAUD ?= 115200
PORT ?= $(shell ls /dev/cu.usbmodem* 2>/dev/null | head -n 1)
PYBULLET_PY ?= .conda-pybullet/bin/python

.PHONY: build upload monitor flash ports clean view-urdf render-urdf fusion-config fusion-visual fusion-jointed view-fusion render-fusion

build:
	arduino-cli compile --fqbn $(FQBN) $(PROJ)

upload:
	@if [ -z "$(PORT)" ]; then \
		echo "No /dev/cu.usbmodem* port found"; \
		echo "Run 'make ports' and retry with PORT=/dev/cu.xxx"; \
		exit 1; \
	fi
	@echo "Using port: $(PORT)"
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) $(PROJ)

monitor:
	@if [ -z "$(PORT)" ]; then \
		echo "No /dev/cu.usbmodem* port found"; \
		echo "Run 'make ports' and retry with PORT=/dev/cu.xxx"; \
		exit 1; \
	fi
	@echo "Using port: $(PORT)"
	arduino-cli monitor -p $(PORT) -c baudrate=$(BAUD)

flash: build upload

ports:
	arduino-cli board list

clean:
	arduino-cli cache clean

view-urdf:
	$(PYBULLET_PY) tools/view_urdf_pybullet.py

render-urdf:
	$(PYBULLET_PY) tools/render_urdf_pybullet.py

fusion-config:
	python3 tools/create_visual_link_config.py

fusion-visual:
	$(PYBULLET_PY) tools/build_fusion_visual_urdf.py

fusion-jointed:
	$(PYBULLET_PY) tools/build_fusion_jointed_urdf.py

view-fusion: fusion-jointed
	$(PYBULLET_PY) tools/view_urdf_pybullet.py --urdf urdf/robot_arm_fusion.urdf

render-fusion: fusion-jointed
	$(PYBULLET_PY) tools/render_urdf_pybullet.py --urdf urdf/robot_arm_fusion.urdf --output outputs/robot_arm_fusion_rebuilt.png
