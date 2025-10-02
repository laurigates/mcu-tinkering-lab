# Pluggable AI Backend Architecture

This document outlines the architecture of the pluggable AI backend system implemented in the `esp32-cam-idf` project. This system was designed to provide flexibility in choosing an AI vision service, allowing for easy extension to support new backends beyond the initial Claude and Ollama implementations.

## Motivation

The original implementation was tightly coupled with the Claude API. This presented two main challenges:
1.  **Vendor Lock-in**: The system was entirely dependent on a single, proprietary AI service.
2.  **Lack of Extensibility**: Adding a new AI service would have required significant code changes throughout the application.

The goal of this refactoring was to create a modular, extensible system where AI backends can be treated as interchangeable components.

## Core Components

The pluggable AI backend is built around a few key components that work together to abstract the specific AI implementation from the main application logic.

### 1. The Generic AI Interface (`ai_backend.h`)

This is the cornerstone of the new architecture. It defines a generic interface that all AI backends must implement. The interface consists of a struct, `ai_backend_t`, which is a collection of function pointers for the core operations of an AI backend:

- `init()`: Initializes the backend with a given configuration (API keys, URLs, etc.).
- `analyze_image()`: Sends an image to the backend for analysis and returns a standardized response.
- `free_response()`: Frees any memory allocated for the response.
- `deinit()`: Deinitializes the backend and releases resources.

It also defines a generic `ai_response_t` struct to ensure that the main application receives data in a consistent format, regardless of the backend used.

### 2. Backend Implementations (`claude_backend.c`, `ollama_backend.c`)

Each supported AI service has its own implementation file that provides the concrete logic for the functions defined in the `ai_backend_t` interface. For example:

- **`claude_backend.c`**: Handles the specifics of authenticating with the Claude API, formatting the JSON request for a vision query, and parsing the response from Claude's servers.
- **`ollama_backend.c`**: Handles sending a request to a self-hosted Ollama server, including base64-encoding the image and constructing the appropriate JSON payload for the `/api/generate` endpoint.

Each implementation also has a corresponding header file (`claude_backend.h`, `ollama_backend.h`) that exposes a single function, `claude_backend_get()` or `ollama_backend_get()`, which returns a pointer to its static `ai_backend_t` interface struct.

### 3. The Backend Selector (`ai_backend.c`)

This file is responsible for selecting the active AI backend at compile time. It uses preprocessor directives (`#if defined(...)`) to check for configuration flags set in `config.h`. Based on the selected flag (`CONFIG_AI_BACKEND_CLAUDE` or `CONFIG_AI_BACKEND_OLLAMA`), the `ai_backend_get_current()` function returns a pointer to the appropriate backend interface.

This is the only part of the system that needs to be aware of all the available backends. The rest of the application interacts only with the generic interface.

### 4. Configuration (`config.h`)

The `config.h` file now contains the selection flags and all necessary configuration details for each backend, such as API URLs and model names. This centralizes the configuration and makes it easy for you to switch between backends and update their settings.

## How to Add a New AI Backend

The new architecture makes it straightforward to add support for another AI service. Here’s how you would do it:

1.  **Create `new_backend.c` and `new_backend.h`**:
    - In `new_backend.c`, implement the functions required by the `ai_backend_t` interface (`init`, `analyze_image`, etc.).
    - Create a static, `const` instance of the `ai_backend_t` struct and populate it with your new functions.
    - In `new_backend.h`, declare a `new_backend_get()` function that returns a pointer to your static backend interface.

2.  **Update the Backend Selector**:
    - In `ai_backend.c`, add a new `#elif defined(CONFIG_AI_BACKEND_NEW)` block to include your new header and return your backend from `ai_backend_get_current()`.

3.  **Update Configuration**:
    - In `config.h`, add a `CONFIG_AI_BACKEND_NEW` definition and any API URLs, keys, or other settings your new backend requires.

4.  **Update the Build System**:
    - In `main/CMakeLists.txt`, add your `new_backend.c` to the list of source files to be compiled.

By following these steps, you can integrate a new AI service without modifying any of the core application logic in `main.c`.
