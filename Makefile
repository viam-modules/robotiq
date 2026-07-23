BIN_OUTPUT_PATH = bin
TOOL_BIN = bin/gotools/$(shell uname -s)-$(shell uname -m)
GOVERSION = $(shell grep '^go .\..' go.mod | head -n1 | cut -d' ' -f2)
COMMON_LDFLAGS = -s -w #-X 'go.viam.com/rdk/config.Version=${TAG_VERSION}' -X 'go.viam.com/rdk/config.GitRevision=${GIT_REVISION}' -X 'go.viam.com/rdk/config.DateCompiled=${DATE_COMPILED}'
UNAME_S ?= $(shell uname -s)

ifeq ($(shell command -v dpkg >/dev/null && dpkg --print-architecture),armhf)
GOFLAGS += -tags=no_tflite
endif

module: build
	rm -f $(BIN_OUTPUT_PATH)/module.tar.gz
	tar czf $(BIN_OUTPUT_PATH)/module.tar.gz $(BIN_OUTPUT_PATH)/robotiq meta.json

build: build-go

build-go:
	rm -f $(BIN_OUTPUT_PATH)/robotiq
	go build -tags no_cgo,osusergo,netgo -ldflags="-extldflags=-static $(COMMON_LDFLAGS)"  -o $(BIN_OUTPUT_PATH)/robotiq main.go

tool-install:
	GOBIN=`pwd`/$(TOOL_BIN) go install \
		github.com/edaniels/golinters/cmd/combined \
		github.com/AlekSi/gocov-xml \
		github.com/axw/gocov/gocov \
		gotest.tools/gotestsum \
		github.com/rhysd/actionlint/cmd/actionlint

lint: lint-go
	PATH=$(TOOL_BIN) actionlint

lint-go: tool-install
	go mod tidy
	export pkgs="`go list -f '{{.Dir}}' ./... | grep -v /proto/`" && echo "$$pkgs" | xargs go vet -vettool=$(TOOL_BIN)/combined
	GOTOOLCHAIN=go$(GOVERSION) GOGC=50 go run github.com/golangci/golangci-lint/cmd/golangci-lint@v1.62.2 run -v --fix --config=./etc/golangci.yaml --timeout=5m

test: test-go

test-go: tool-install
	go test -race ./...

clean-all:
	git clean -fxd

license-check:
	license_finder