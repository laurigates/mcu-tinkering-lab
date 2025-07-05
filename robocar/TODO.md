✅ **COMPLETED** - Added starting ollama to a make target
- Added `make ollama-start` - starts Ollama with OLLAMA_HOST=0.0.0.0
- Added `make ollama-advertise` - publishes mDNS service discovery
- Added `make ollama-setup` - does both start and advertise
- Added `make ollama-stop` - stops all Ollama services

✅ **COMPLETED** - Added complete develop targets
- Fixed `make develop-main` - now does build + flash + monitor
- Fixed `make develop-cam` - now does build + flash + monitor
- Both targets provide complete development workflow in one command
