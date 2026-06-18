"use strict";
(() => {
  var __create = Object.create;
  var __defProp = Object.defineProperty;
  var __getOwnPropDesc = Object.getOwnPropertyDescriptor;
  var __getOwnPropNames = Object.getOwnPropertyNames;
  var __getProtoOf = Object.getPrototypeOf;
  var __hasOwnProp = Object.prototype.hasOwnProperty;
  var __commonJS = (cb, mod) => function __require() {
    return mod || (0, cb[__getOwnPropNames(cb)[0]])((mod = { exports: {} }).exports, mod), mod.exports;
  };
  var __export = (target, all) => {
    for (var name in all)
      __defProp(target, name, { get: all[name], enumerable: true });
  };
  var __copyProps = (to, from, except, desc) => {
    if (from && typeof from === "object" || typeof from === "function") {
      for (let key of __getOwnPropNames(from))
        if (!__hasOwnProp.call(to, key) && key !== except)
          __defProp(to, key, { get: () => from[key], enumerable: !(desc = __getOwnPropDesc(from, key)) || desc.enumerable });
    }
    return to;
  };
  var __toESM = (mod, isNodeMode, target) => (target = mod != null ? __create(__getProtoOf(mod)) : {}, __copyProps(
    // If the importer is in node compatibility mode or this is not an ESM
    // file that has been converted to a CommonJS file using a Babel-
    // compatible transform (i.e. "__esModule" has not been set), then set
    // "default" to the CommonJS "module.exports" for node compatibility.
    isNodeMode || !mod || !mod.__esModule ? __defProp(target, "default", { value: mod, enumerable: true }) : target,
    mod
  ));

  // node_modules/base64-js/index.js
  var require_base64_js = __commonJS({
    "node_modules/base64-js/index.js"(exports) {
      "use strict";
      exports.byteLength = byteLength;
      exports.toByteArray = toByteArray;
      exports.fromByteArray = fromByteArray;
      var lookup = [];
      var revLookup = [];
      var Arr = typeof Uint8Array !== "undefined" ? Uint8Array : Array;
      var code = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
      for (i = 0, len = code.length; i < len; ++i) {
        lookup[i] = code[i];
        revLookup[code.charCodeAt(i)] = i;
      }
      var i;
      var len;
      revLookup["-".charCodeAt(0)] = 62;
      revLookup["_".charCodeAt(0)] = 63;
      function getLens(b64) {
        var len2 = b64.length;
        if (len2 % 4 > 0) {
          throw new Error("Invalid string. Length must be a multiple of 4");
        }
        var validLen = b64.indexOf("=");
        if (validLen === -1) validLen = len2;
        var placeHoldersLen = validLen === len2 ? 0 : 4 - validLen % 4;
        return [validLen, placeHoldersLen];
      }
      function byteLength(b64) {
        var lens = getLens(b64);
        var validLen = lens[0];
        var placeHoldersLen = lens[1];
        return (validLen + placeHoldersLen) * 3 / 4 - placeHoldersLen;
      }
      function _byteLength(b64, validLen, placeHoldersLen) {
        return (validLen + placeHoldersLen) * 3 / 4 - placeHoldersLen;
      }
      function toByteArray(b64) {
        var tmp;
        var lens = getLens(b64);
        var validLen = lens[0];
        var placeHoldersLen = lens[1];
        var arr = new Arr(_byteLength(b64, validLen, placeHoldersLen));
        var curByte = 0;
        var len2 = placeHoldersLen > 0 ? validLen - 4 : validLen;
        var i2;
        for (i2 = 0; i2 < len2; i2 += 4) {
          tmp = revLookup[b64.charCodeAt(i2)] << 18 | revLookup[b64.charCodeAt(i2 + 1)] << 12 | revLookup[b64.charCodeAt(i2 + 2)] << 6 | revLookup[b64.charCodeAt(i2 + 3)];
          arr[curByte++] = tmp >> 16 & 255;
          arr[curByte++] = tmp >> 8 & 255;
          arr[curByte++] = tmp & 255;
        }
        if (placeHoldersLen === 2) {
          tmp = revLookup[b64.charCodeAt(i2)] << 2 | revLookup[b64.charCodeAt(i2 + 1)] >> 4;
          arr[curByte++] = tmp & 255;
        }
        if (placeHoldersLen === 1) {
          tmp = revLookup[b64.charCodeAt(i2)] << 10 | revLookup[b64.charCodeAt(i2 + 1)] << 4 | revLookup[b64.charCodeAt(i2 + 2)] >> 2;
          arr[curByte++] = tmp >> 8 & 255;
          arr[curByte++] = tmp & 255;
        }
        return arr;
      }
      function tripletToBase64(num) {
        return lookup[num >> 18 & 63] + lookup[num >> 12 & 63] + lookup[num >> 6 & 63] + lookup[num & 63];
      }
      function encodeChunk(uint8, start, end) {
        var tmp;
        var output = [];
        for (var i2 = start; i2 < end; i2 += 3) {
          tmp = (uint8[i2] << 16 & 16711680) + (uint8[i2 + 1] << 8 & 65280) + (uint8[i2 + 2] & 255);
          output.push(tripletToBase64(tmp));
        }
        return output.join("");
      }
      function fromByteArray(uint8) {
        var tmp;
        var len2 = uint8.length;
        var extraBytes = len2 % 3;
        var parts = [];
        var maxChunkLength = 16383;
        for (var i2 = 0, len22 = len2 - extraBytes; i2 < len22; i2 += maxChunkLength) {
          parts.push(encodeChunk(uint8, i2, i2 + maxChunkLength > len22 ? len22 : i2 + maxChunkLength));
        }
        if (extraBytes === 1) {
          tmp = uint8[len2 - 1];
          parts.push(
            lookup[tmp >> 2] + lookup[tmp << 4 & 63] + "=="
          );
        } else if (extraBytes === 2) {
          tmp = (uint8[len2 - 2] << 8) + uint8[len2 - 1];
          parts.push(
            lookup[tmp >> 10] + lookup[tmp >> 4 & 63] + lookup[tmp << 2 & 63] + "="
          );
        }
        return parts.join("");
      }
    }
  });

  // node_modules/ieee754/index.js
  var require_ieee754 = __commonJS({
    "node_modules/ieee754/index.js"(exports) {
      exports.read = function(buffer, offset, isLE, mLen, nBytes) {
        var e, m;
        var eLen = nBytes * 8 - mLen - 1;
        var eMax = (1 << eLen) - 1;
        var eBias = eMax >> 1;
        var nBits = -7;
        var i = isLE ? nBytes - 1 : 0;
        var d = isLE ? -1 : 1;
        var s = buffer[offset + i];
        i += d;
        e = s & (1 << -nBits) - 1;
        s >>= -nBits;
        nBits += eLen;
        for (; nBits > 0; e = e * 256 + buffer[offset + i], i += d, nBits -= 8) {
        }
        m = e & (1 << -nBits) - 1;
        e >>= -nBits;
        nBits += mLen;
        for (; nBits > 0; m = m * 256 + buffer[offset + i], i += d, nBits -= 8) {
        }
        if (e === 0) {
          e = 1 - eBias;
        } else if (e === eMax) {
          return m ? NaN : (s ? -1 : 1) * Infinity;
        } else {
          m = m + Math.pow(2, mLen);
          e = e - eBias;
        }
        return (s ? -1 : 1) * m * Math.pow(2, e - mLen);
      };
      exports.write = function(buffer, value, offset, isLE, mLen, nBytes) {
        var e, m, c;
        var eLen = nBytes * 8 - mLen - 1;
        var eMax = (1 << eLen) - 1;
        var eBias = eMax >> 1;
        var rt = mLen === 23 ? Math.pow(2, -24) - Math.pow(2, -77) : 0;
        var i = isLE ? 0 : nBytes - 1;
        var d = isLE ? 1 : -1;
        var s = value < 0 || value === 0 && 1 / value < 0 ? 1 : 0;
        value = Math.abs(value);
        if (isNaN(value) || value === Infinity) {
          m = isNaN(value) ? 1 : 0;
          e = eMax;
        } else {
          e = Math.floor(Math.log(value) / Math.LN2);
          if (value * (c = Math.pow(2, -e)) < 1) {
            e--;
            c *= 2;
          }
          if (e + eBias >= 1) {
            value += rt / c;
          } else {
            value += rt * Math.pow(2, 1 - eBias);
          }
          if (value * c >= 2) {
            e++;
            c /= 2;
          }
          if (e + eBias >= eMax) {
            m = 0;
            e = eMax;
          } else if (e + eBias >= 1) {
            m = (value * c - 1) * Math.pow(2, mLen);
            e = e + eBias;
          } else {
            m = value * Math.pow(2, eBias - 1) * Math.pow(2, mLen);
            e = 0;
          }
        }
        for (; mLen >= 8; buffer[offset + i] = m & 255, i += d, m /= 256, mLen -= 8) {
        }
        e = e << mLen | m;
        eLen += mLen;
        for (; eLen > 0; buffer[offset + i] = e & 255, i += d, e /= 256, eLen -= 8) {
        }
        buffer[offset + i - d] |= s * 128;
      };
    }
  });

  // node_modules/buffer/index.js
  var require_buffer = __commonJS({
    "node_modules/buffer/index.js"(exports) {
      "use strict";
      var base64 = require_base64_js();
      var ieee754 = require_ieee754();
      var customInspectSymbol = typeof Symbol === "function" && typeof Symbol["for"] === "function" ? Symbol["for"]("nodejs.util.inspect.custom") : null;
      exports.Buffer = Buffer3;
      exports.SlowBuffer = SlowBuffer;
      exports.INSPECT_MAX_BYTES = 50;
      var K_MAX_LENGTH = 2147483647;
      exports.kMaxLength = K_MAX_LENGTH;
      Buffer3.TYPED_ARRAY_SUPPORT = typedArraySupport();
      if (!Buffer3.TYPED_ARRAY_SUPPORT && typeof console !== "undefined" && typeof console.error === "function") {
        console.error(
          "This browser lacks typed array (Uint8Array) support which is required by `buffer` v5.x. Use `buffer` v4.x if you require old browser support."
        );
      }
      function typedArraySupport() {
        try {
          const arr = new Uint8Array(1);
          const proto = { foo: function() {
            return 42;
          } };
          Object.setPrototypeOf(proto, Uint8Array.prototype);
          Object.setPrototypeOf(arr, proto);
          return arr.foo() === 42;
        } catch (e) {
          return false;
        }
      }
      Object.defineProperty(Buffer3.prototype, "parent", {
        enumerable: true,
        get: function() {
          if (!Buffer3.isBuffer(this)) return void 0;
          return this.buffer;
        }
      });
      Object.defineProperty(Buffer3.prototype, "offset", {
        enumerable: true,
        get: function() {
          if (!Buffer3.isBuffer(this)) return void 0;
          return this.byteOffset;
        }
      });
      function createBuffer(length) {
        if (length > K_MAX_LENGTH) {
          throw new RangeError('The value "' + length + '" is invalid for option "size"');
        }
        const buf = new Uint8Array(length);
        Object.setPrototypeOf(buf, Buffer3.prototype);
        return buf;
      }
      function Buffer3(arg, encodingOrOffset, length) {
        if (typeof arg === "number") {
          if (typeof encodingOrOffset === "string") {
            throw new TypeError(
              'The "string" argument must be of type string. Received type number'
            );
          }
          return allocUnsafe(arg);
        }
        return from(arg, encodingOrOffset, length);
      }
      Buffer3.poolSize = 8192;
      function from(value, encodingOrOffset, length) {
        if (typeof value === "string") {
          return fromString(value, encodingOrOffset);
        }
        if (ArrayBuffer.isView(value)) {
          return fromArrayView(value);
        }
        if (value == null) {
          throw new TypeError(
            "The first argument must be one of type string, Buffer, ArrayBuffer, Array, or Array-like Object. Received type " + typeof value
          );
        }
        if (isInstance(value, ArrayBuffer) || value && isInstance(value.buffer, ArrayBuffer)) {
          return fromArrayBuffer(value, encodingOrOffset, length);
        }
        if (typeof SharedArrayBuffer !== "undefined" && (isInstance(value, SharedArrayBuffer) || value && isInstance(value.buffer, SharedArrayBuffer))) {
          return fromArrayBuffer(value, encodingOrOffset, length);
        }
        if (typeof value === "number") {
          throw new TypeError(
            'The "value" argument must not be of type number. Received type number'
          );
        }
        const valueOf = value.valueOf && value.valueOf();
        if (valueOf != null && valueOf !== value) {
          return Buffer3.from(valueOf, encodingOrOffset, length);
        }
        const b = fromObject(value);
        if (b) return b;
        if (typeof Symbol !== "undefined" && Symbol.toPrimitive != null && typeof value[Symbol.toPrimitive] === "function") {
          return Buffer3.from(value[Symbol.toPrimitive]("string"), encodingOrOffset, length);
        }
        throw new TypeError(
          "The first argument must be one of type string, Buffer, ArrayBuffer, Array, or Array-like Object. Received type " + typeof value
        );
      }
      Buffer3.from = function(value, encodingOrOffset, length) {
        return from(value, encodingOrOffset, length);
      };
      Object.setPrototypeOf(Buffer3.prototype, Uint8Array.prototype);
      Object.setPrototypeOf(Buffer3, Uint8Array);
      function assertSize(size) {
        if (typeof size !== "number") {
          throw new TypeError('"size" argument must be of type number');
        } else if (size < 0) {
          throw new RangeError('The value "' + size + '" is invalid for option "size"');
        }
      }
      function alloc(size, fill, encoding) {
        assertSize(size);
        if (size <= 0) {
          return createBuffer(size);
        }
        if (fill !== void 0) {
          return typeof encoding === "string" ? createBuffer(size).fill(fill, encoding) : createBuffer(size).fill(fill);
        }
        return createBuffer(size);
      }
      Buffer3.alloc = function(size, fill, encoding) {
        return alloc(size, fill, encoding);
      };
      function allocUnsafe(size) {
        assertSize(size);
        return createBuffer(size < 0 ? 0 : checked(size) | 0);
      }
      Buffer3.allocUnsafe = function(size) {
        return allocUnsafe(size);
      };
      Buffer3.allocUnsafeSlow = function(size) {
        return allocUnsafe(size);
      };
      function fromString(string, encoding) {
        if (typeof encoding !== "string" || encoding === "") {
          encoding = "utf8";
        }
        if (!Buffer3.isEncoding(encoding)) {
          throw new TypeError("Unknown encoding: " + encoding);
        }
        const length = byteLength(string, encoding) | 0;
        let buf = createBuffer(length);
        const actual = buf.write(string, encoding);
        if (actual !== length) {
          buf = buf.slice(0, actual);
        }
        return buf;
      }
      function fromArrayLike(array) {
        const length = array.length < 0 ? 0 : checked(array.length) | 0;
        const buf = createBuffer(length);
        for (let i = 0; i < length; i += 1) {
          buf[i] = array[i] & 255;
        }
        return buf;
      }
      function fromArrayView(arrayView) {
        if (isInstance(arrayView, Uint8Array)) {
          const copy = new Uint8Array(arrayView);
          return fromArrayBuffer(copy.buffer, copy.byteOffset, copy.byteLength);
        }
        return fromArrayLike(arrayView);
      }
      function fromArrayBuffer(array, byteOffset, length) {
        if (byteOffset < 0 || array.byteLength < byteOffset) {
          throw new RangeError('"offset" is outside of buffer bounds');
        }
        if (array.byteLength < byteOffset + (length || 0)) {
          throw new RangeError('"length" is outside of buffer bounds');
        }
        let buf;
        if (byteOffset === void 0 && length === void 0) {
          buf = new Uint8Array(array);
        } else if (length === void 0) {
          buf = new Uint8Array(array, byteOffset);
        } else {
          buf = new Uint8Array(array, byteOffset, length);
        }
        Object.setPrototypeOf(buf, Buffer3.prototype);
        return buf;
      }
      function fromObject(obj) {
        if (Buffer3.isBuffer(obj)) {
          const len = checked(obj.length) | 0;
          const buf = createBuffer(len);
          if (buf.length === 0) {
            return buf;
          }
          obj.copy(buf, 0, 0, len);
          return buf;
        }
        if (obj.length !== void 0) {
          if (typeof obj.length !== "number" || numberIsNaN(obj.length)) {
            return createBuffer(0);
          }
          return fromArrayLike(obj);
        }
        if (obj.type === "Buffer" && Array.isArray(obj.data)) {
          return fromArrayLike(obj.data);
        }
      }
      function checked(length) {
        if (length >= K_MAX_LENGTH) {
          throw new RangeError("Attempt to allocate Buffer larger than maximum size: 0x" + K_MAX_LENGTH.toString(16) + " bytes");
        }
        return length | 0;
      }
      function SlowBuffer(length) {
        if (+length != length) {
          length = 0;
        }
        return Buffer3.alloc(+length);
      }
      Buffer3.isBuffer = function isBuffer(b) {
        return b != null && b._isBuffer === true && b !== Buffer3.prototype;
      };
      Buffer3.compare = function compare(a, b) {
        if (isInstance(a, Uint8Array)) a = Buffer3.from(a, a.offset, a.byteLength);
        if (isInstance(b, Uint8Array)) b = Buffer3.from(b, b.offset, b.byteLength);
        if (!Buffer3.isBuffer(a) || !Buffer3.isBuffer(b)) {
          throw new TypeError(
            'The "buf1", "buf2" arguments must be one of type Buffer or Uint8Array'
          );
        }
        if (a === b) return 0;
        let x = a.length;
        let y = b.length;
        for (let i = 0, len = Math.min(x, y); i < len; ++i) {
          if (a[i] !== b[i]) {
            x = a[i];
            y = b[i];
            break;
          }
        }
        if (x < y) return -1;
        if (y < x) return 1;
        return 0;
      };
      Buffer3.isEncoding = function isEncoding(encoding) {
        switch (String(encoding).toLowerCase()) {
          case "hex":
          case "utf8":
          case "utf-8":
          case "ascii":
          case "latin1":
          case "binary":
          case "base64":
          case "ucs2":
          case "ucs-2":
          case "utf16le":
          case "utf-16le":
            return true;
          default:
            return false;
        }
      };
      Buffer3.concat = function concat(list, length) {
        if (!Array.isArray(list)) {
          throw new TypeError('"list" argument must be an Array of Buffers');
        }
        if (list.length === 0) {
          return Buffer3.alloc(0);
        }
        let i;
        if (length === void 0) {
          length = 0;
          for (i = 0; i < list.length; ++i) {
            length += list[i].length;
          }
        }
        const buffer = Buffer3.allocUnsafe(length);
        let pos = 0;
        for (i = 0; i < list.length; ++i) {
          let buf = list[i];
          if (isInstance(buf, Uint8Array)) {
            if (pos + buf.length > buffer.length) {
              if (!Buffer3.isBuffer(buf)) buf = Buffer3.from(buf);
              buf.copy(buffer, pos);
            } else {
              Uint8Array.prototype.set.call(
                buffer,
                buf,
                pos
              );
            }
          } else if (!Buffer3.isBuffer(buf)) {
            throw new TypeError('"list" argument must be an Array of Buffers');
          } else {
            buf.copy(buffer, pos);
          }
          pos += buf.length;
        }
        return buffer;
      };
      function byteLength(string, encoding) {
        if (Buffer3.isBuffer(string)) {
          return string.length;
        }
        if (ArrayBuffer.isView(string) || isInstance(string, ArrayBuffer)) {
          return string.byteLength;
        }
        if (typeof string !== "string") {
          throw new TypeError(
            'The "string" argument must be one of type string, Buffer, or ArrayBuffer. Received type ' + typeof string
          );
        }
        const len = string.length;
        const mustMatch = arguments.length > 2 && arguments[2] === true;
        if (!mustMatch && len === 0) return 0;
        let loweredCase = false;
        for (; ; ) {
          switch (encoding) {
            case "ascii":
            case "latin1":
            case "binary":
              return len;
            case "utf8":
            case "utf-8":
              return utf8ToBytes(string).length;
            case "ucs2":
            case "ucs-2":
            case "utf16le":
            case "utf-16le":
              return len * 2;
            case "hex":
              return len >>> 1;
            case "base64":
              return base64ToBytes(string).length;
            default:
              if (loweredCase) {
                return mustMatch ? -1 : utf8ToBytes(string).length;
              }
              encoding = ("" + encoding).toLowerCase();
              loweredCase = true;
          }
        }
      }
      Buffer3.byteLength = byteLength;
      function slowToString(encoding, start, end) {
        let loweredCase = false;
        if (start === void 0 || start < 0) {
          start = 0;
        }
        if (start > this.length) {
          return "";
        }
        if (end === void 0 || end > this.length) {
          end = this.length;
        }
        if (end <= 0) {
          return "";
        }
        end >>>= 0;
        start >>>= 0;
        if (end <= start) {
          return "";
        }
        if (!encoding) encoding = "utf8";
        while (true) {
          switch (encoding) {
            case "hex":
              return hexSlice(this, start, end);
            case "utf8":
            case "utf-8":
              return utf8Slice(this, start, end);
            case "ascii":
              return asciiSlice(this, start, end);
            case "latin1":
            case "binary":
              return latin1Slice(this, start, end);
            case "base64":
              return base64Slice(this, start, end);
            case "ucs2":
            case "ucs-2":
            case "utf16le":
            case "utf-16le":
              return utf16leSlice(this, start, end);
            default:
              if (loweredCase) throw new TypeError("Unknown encoding: " + encoding);
              encoding = (encoding + "").toLowerCase();
              loweredCase = true;
          }
        }
      }
      Buffer3.prototype._isBuffer = true;
      function swap(b, n, m) {
        const i = b[n];
        b[n] = b[m];
        b[m] = i;
      }
      Buffer3.prototype.swap16 = function swap16() {
        const len = this.length;
        if (len % 2 !== 0) {
          throw new RangeError("Buffer size must be a multiple of 16-bits");
        }
        for (let i = 0; i < len; i += 2) {
          swap(this, i, i + 1);
        }
        return this;
      };
      Buffer3.prototype.swap32 = function swap32() {
        const len = this.length;
        if (len % 4 !== 0) {
          throw new RangeError("Buffer size must be a multiple of 32-bits");
        }
        for (let i = 0; i < len; i += 4) {
          swap(this, i, i + 3);
          swap(this, i + 1, i + 2);
        }
        return this;
      };
      Buffer3.prototype.swap64 = function swap64() {
        const len = this.length;
        if (len % 8 !== 0) {
          throw new RangeError("Buffer size must be a multiple of 64-bits");
        }
        for (let i = 0; i < len; i += 8) {
          swap(this, i, i + 7);
          swap(this, i + 1, i + 6);
          swap(this, i + 2, i + 5);
          swap(this, i + 3, i + 4);
        }
        return this;
      };
      Buffer3.prototype.toString = function toString() {
        const length = this.length;
        if (length === 0) return "";
        if (arguments.length === 0) return utf8Slice(this, 0, length);
        return slowToString.apply(this, arguments);
      };
      Buffer3.prototype.toLocaleString = Buffer3.prototype.toString;
      Buffer3.prototype.equals = function equals(b) {
        if (!Buffer3.isBuffer(b)) throw new TypeError("Argument must be a Buffer");
        if (this === b) return true;
        return Buffer3.compare(this, b) === 0;
      };
      Buffer3.prototype.inspect = function inspect() {
        let str = "";
        const max = exports.INSPECT_MAX_BYTES;
        str = this.toString("hex", 0, max).replace(/(.{2})/g, "$1 ").trim();
        if (this.length > max) str += " ... ";
        return "<Buffer " + str + ">";
      };
      if (customInspectSymbol) {
        Buffer3.prototype[customInspectSymbol] = Buffer3.prototype.inspect;
      }
      Buffer3.prototype.compare = function compare(target, start, end, thisStart, thisEnd) {
        if (isInstance(target, Uint8Array)) {
          target = Buffer3.from(target, target.offset, target.byteLength);
        }
        if (!Buffer3.isBuffer(target)) {
          throw new TypeError(
            'The "target" argument must be one of type Buffer or Uint8Array. Received type ' + typeof target
          );
        }
        if (start === void 0) {
          start = 0;
        }
        if (end === void 0) {
          end = target ? target.length : 0;
        }
        if (thisStart === void 0) {
          thisStart = 0;
        }
        if (thisEnd === void 0) {
          thisEnd = this.length;
        }
        if (start < 0 || end > target.length || thisStart < 0 || thisEnd > this.length) {
          throw new RangeError("out of range index");
        }
        if (thisStart >= thisEnd && start >= end) {
          return 0;
        }
        if (thisStart >= thisEnd) {
          return -1;
        }
        if (start >= end) {
          return 1;
        }
        start >>>= 0;
        end >>>= 0;
        thisStart >>>= 0;
        thisEnd >>>= 0;
        if (this === target) return 0;
        let x = thisEnd - thisStart;
        let y = end - start;
        const len = Math.min(x, y);
        const thisCopy = this.slice(thisStart, thisEnd);
        const targetCopy = target.slice(start, end);
        for (let i = 0; i < len; ++i) {
          if (thisCopy[i] !== targetCopy[i]) {
            x = thisCopy[i];
            y = targetCopy[i];
            break;
          }
        }
        if (x < y) return -1;
        if (y < x) return 1;
        return 0;
      };
      function bidirectionalIndexOf(buffer, val, byteOffset, encoding, dir) {
        if (buffer.length === 0) return -1;
        if (typeof byteOffset === "string") {
          encoding = byteOffset;
          byteOffset = 0;
        } else if (byteOffset > 2147483647) {
          byteOffset = 2147483647;
        } else if (byteOffset < -2147483648) {
          byteOffset = -2147483648;
        }
        byteOffset = +byteOffset;
        if (numberIsNaN(byteOffset)) {
          byteOffset = dir ? 0 : buffer.length - 1;
        }
        if (byteOffset < 0) byteOffset = buffer.length + byteOffset;
        if (byteOffset >= buffer.length) {
          if (dir) return -1;
          else byteOffset = buffer.length - 1;
        } else if (byteOffset < 0) {
          if (dir) byteOffset = 0;
          else return -1;
        }
        if (typeof val === "string") {
          val = Buffer3.from(val, encoding);
        }
        if (Buffer3.isBuffer(val)) {
          if (val.length === 0) {
            return -1;
          }
          return arrayIndexOf(buffer, val, byteOffset, encoding, dir);
        } else if (typeof val === "number") {
          val = val & 255;
          if (typeof Uint8Array.prototype.indexOf === "function") {
            if (dir) {
              return Uint8Array.prototype.indexOf.call(buffer, val, byteOffset);
            } else {
              return Uint8Array.prototype.lastIndexOf.call(buffer, val, byteOffset);
            }
          }
          return arrayIndexOf(buffer, [val], byteOffset, encoding, dir);
        }
        throw new TypeError("val must be string, number or Buffer");
      }
      function arrayIndexOf(arr, val, byteOffset, encoding, dir) {
        let indexSize = 1;
        let arrLength = arr.length;
        let valLength = val.length;
        if (encoding !== void 0) {
          encoding = String(encoding).toLowerCase();
          if (encoding === "ucs2" || encoding === "ucs-2" || encoding === "utf16le" || encoding === "utf-16le") {
            if (arr.length < 2 || val.length < 2) {
              return -1;
            }
            indexSize = 2;
            arrLength /= 2;
            valLength /= 2;
            byteOffset /= 2;
          }
        }
        function read(buf, i2) {
          if (indexSize === 1) {
            return buf[i2];
          } else {
            return buf.readUInt16BE(i2 * indexSize);
          }
        }
        let i;
        if (dir) {
          let foundIndex = -1;
          for (i = byteOffset; i < arrLength; i++) {
            if (read(arr, i) === read(val, foundIndex === -1 ? 0 : i - foundIndex)) {
              if (foundIndex === -1) foundIndex = i;
              if (i - foundIndex + 1 === valLength) return foundIndex * indexSize;
            } else {
              if (foundIndex !== -1) i -= i - foundIndex;
              foundIndex = -1;
            }
          }
        } else {
          if (byteOffset + valLength > arrLength) byteOffset = arrLength - valLength;
          for (i = byteOffset; i >= 0; i--) {
            let found = true;
            for (let j = 0; j < valLength; j++) {
              if (read(arr, i + j) !== read(val, j)) {
                found = false;
                break;
              }
            }
            if (found) return i;
          }
        }
        return -1;
      }
      Buffer3.prototype.includes = function includes(val, byteOffset, encoding) {
        return this.indexOf(val, byteOffset, encoding) !== -1;
      };
      Buffer3.prototype.indexOf = function indexOf(val, byteOffset, encoding) {
        return bidirectionalIndexOf(this, val, byteOffset, encoding, true);
      };
      Buffer3.prototype.lastIndexOf = function lastIndexOf(val, byteOffset, encoding) {
        return bidirectionalIndexOf(this, val, byteOffset, encoding, false);
      };
      function hexWrite(buf, string, offset, length) {
        offset = Number(offset) || 0;
        const remaining = buf.length - offset;
        if (!length) {
          length = remaining;
        } else {
          length = Number(length);
          if (length > remaining) {
            length = remaining;
          }
        }
        const strLen = string.length;
        if (length > strLen / 2) {
          length = strLen / 2;
        }
        let i;
        for (i = 0; i < length; ++i) {
          const parsed = parseInt(string.substr(i * 2, 2), 16);
          if (numberIsNaN(parsed)) return i;
          buf[offset + i] = parsed;
        }
        return i;
      }
      function utf8Write(buf, string, offset, length) {
        return blitBuffer(utf8ToBytes(string, buf.length - offset), buf, offset, length);
      }
      function asciiWrite(buf, string, offset, length) {
        return blitBuffer(asciiToBytes(string), buf, offset, length);
      }
      function base64Write(buf, string, offset, length) {
        return blitBuffer(base64ToBytes(string), buf, offset, length);
      }
      function ucs2Write(buf, string, offset, length) {
        return blitBuffer(utf16leToBytes(string, buf.length - offset), buf, offset, length);
      }
      Buffer3.prototype.write = function write(string, offset, length, encoding) {
        if (offset === void 0) {
          encoding = "utf8";
          length = this.length;
          offset = 0;
        } else if (length === void 0 && typeof offset === "string") {
          encoding = offset;
          length = this.length;
          offset = 0;
        } else if (isFinite(offset)) {
          offset = offset >>> 0;
          if (isFinite(length)) {
            length = length >>> 0;
            if (encoding === void 0) encoding = "utf8";
          } else {
            encoding = length;
            length = void 0;
          }
        } else {
          throw new Error(
            "Buffer.write(string, encoding, offset[, length]) is no longer supported"
          );
        }
        const remaining = this.length - offset;
        if (length === void 0 || length > remaining) length = remaining;
        if (string.length > 0 && (length < 0 || offset < 0) || offset > this.length) {
          throw new RangeError("Attempt to write outside buffer bounds");
        }
        if (!encoding) encoding = "utf8";
        let loweredCase = false;
        for (; ; ) {
          switch (encoding) {
            case "hex":
              return hexWrite(this, string, offset, length);
            case "utf8":
            case "utf-8":
              return utf8Write(this, string, offset, length);
            case "ascii":
            case "latin1":
            case "binary":
              return asciiWrite(this, string, offset, length);
            case "base64":
              return base64Write(this, string, offset, length);
            case "ucs2":
            case "ucs-2":
            case "utf16le":
            case "utf-16le":
              return ucs2Write(this, string, offset, length);
            default:
              if (loweredCase) throw new TypeError("Unknown encoding: " + encoding);
              encoding = ("" + encoding).toLowerCase();
              loweredCase = true;
          }
        }
      };
      Buffer3.prototype.toJSON = function toJSON() {
        return {
          type: "Buffer",
          data: Array.prototype.slice.call(this._arr || this, 0)
        };
      };
      function base64Slice(buf, start, end) {
        if (start === 0 && end === buf.length) {
          return base64.fromByteArray(buf);
        } else {
          return base64.fromByteArray(buf.slice(start, end));
        }
      }
      function utf8Slice(buf, start, end) {
        end = Math.min(buf.length, end);
        const res = [];
        let i = start;
        while (i < end) {
          const firstByte = buf[i];
          let codePoint = null;
          let bytesPerSequence = firstByte > 239 ? 4 : firstByte > 223 ? 3 : firstByte > 191 ? 2 : 1;
          if (i + bytesPerSequence <= end) {
            let secondByte, thirdByte, fourthByte, tempCodePoint;
            switch (bytesPerSequence) {
              case 1:
                if (firstByte < 128) {
                  codePoint = firstByte;
                }
                break;
              case 2:
                secondByte = buf[i + 1];
                if ((secondByte & 192) === 128) {
                  tempCodePoint = (firstByte & 31) << 6 | secondByte & 63;
                  if (tempCodePoint > 127) {
                    codePoint = tempCodePoint;
                  }
                }
                break;
              case 3:
                secondByte = buf[i + 1];
                thirdByte = buf[i + 2];
                if ((secondByte & 192) === 128 && (thirdByte & 192) === 128) {
                  tempCodePoint = (firstByte & 15) << 12 | (secondByte & 63) << 6 | thirdByte & 63;
                  if (tempCodePoint > 2047 && (tempCodePoint < 55296 || tempCodePoint > 57343)) {
                    codePoint = tempCodePoint;
                  }
                }
                break;
              case 4:
                secondByte = buf[i + 1];
                thirdByte = buf[i + 2];
                fourthByte = buf[i + 3];
                if ((secondByte & 192) === 128 && (thirdByte & 192) === 128 && (fourthByte & 192) === 128) {
                  tempCodePoint = (firstByte & 15) << 18 | (secondByte & 63) << 12 | (thirdByte & 63) << 6 | fourthByte & 63;
                  if (tempCodePoint > 65535 && tempCodePoint < 1114112) {
                    codePoint = tempCodePoint;
                  }
                }
            }
          }
          if (codePoint === null) {
            codePoint = 65533;
            bytesPerSequence = 1;
          } else if (codePoint > 65535) {
            codePoint -= 65536;
            res.push(codePoint >>> 10 & 1023 | 55296);
            codePoint = 56320 | codePoint & 1023;
          }
          res.push(codePoint);
          i += bytesPerSequence;
        }
        return decodeCodePointsArray(res);
      }
      var MAX_ARGUMENTS_LENGTH = 4096;
      function decodeCodePointsArray(codePoints) {
        const len = codePoints.length;
        if (len <= MAX_ARGUMENTS_LENGTH) {
          return String.fromCharCode.apply(String, codePoints);
        }
        let res = "";
        let i = 0;
        while (i < len) {
          res += String.fromCharCode.apply(
            String,
            codePoints.slice(i, i += MAX_ARGUMENTS_LENGTH)
          );
        }
        return res;
      }
      function asciiSlice(buf, start, end) {
        let ret = "";
        end = Math.min(buf.length, end);
        for (let i = start; i < end; ++i) {
          ret += String.fromCharCode(buf[i] & 127);
        }
        return ret;
      }
      function latin1Slice(buf, start, end) {
        let ret = "";
        end = Math.min(buf.length, end);
        for (let i = start; i < end; ++i) {
          ret += String.fromCharCode(buf[i]);
        }
        return ret;
      }
      function hexSlice(buf, start, end) {
        const len = buf.length;
        if (!start || start < 0) start = 0;
        if (!end || end < 0 || end > len) end = len;
        let out = "";
        for (let i = start; i < end; ++i) {
          out += hexSliceLookupTable[buf[i]];
        }
        return out;
      }
      function utf16leSlice(buf, start, end) {
        const bytes = buf.slice(start, end);
        let res = "";
        for (let i = 0; i < bytes.length - 1; i += 2) {
          res += String.fromCharCode(bytes[i] + bytes[i + 1] * 256);
        }
        return res;
      }
      Buffer3.prototype.slice = function slice(start, end) {
        const len = this.length;
        start = ~~start;
        end = end === void 0 ? len : ~~end;
        if (start < 0) {
          start += len;
          if (start < 0) start = 0;
        } else if (start > len) {
          start = len;
        }
        if (end < 0) {
          end += len;
          if (end < 0) end = 0;
        } else if (end > len) {
          end = len;
        }
        if (end < start) end = start;
        const newBuf = this.subarray(start, end);
        Object.setPrototypeOf(newBuf, Buffer3.prototype);
        return newBuf;
      };
      function checkOffset(offset, ext, length) {
        if (offset % 1 !== 0 || offset < 0) throw new RangeError("offset is not uint");
        if (offset + ext > length) throw new RangeError("Trying to access beyond buffer length");
      }
      Buffer3.prototype.readUintLE = Buffer3.prototype.readUIntLE = function readUIntLE(offset, byteLength2, noAssert) {
        offset = offset >>> 0;
        byteLength2 = byteLength2 >>> 0;
        if (!noAssert) checkOffset(offset, byteLength2, this.length);
        let val = this[offset];
        let mul = 1;
        let i = 0;
        while (++i < byteLength2 && (mul *= 256)) {
          val += this[offset + i] * mul;
        }
        return val;
      };
      Buffer3.prototype.readUintBE = Buffer3.prototype.readUIntBE = function readUIntBE(offset, byteLength2, noAssert) {
        offset = offset >>> 0;
        byteLength2 = byteLength2 >>> 0;
        if (!noAssert) {
          checkOffset(offset, byteLength2, this.length);
        }
        let val = this[offset + --byteLength2];
        let mul = 1;
        while (byteLength2 > 0 && (mul *= 256)) {
          val += this[offset + --byteLength2] * mul;
        }
        return val;
      };
      Buffer3.prototype.readUint8 = Buffer3.prototype.readUInt8 = function readUInt8(offset, noAssert) {
        offset = offset >>> 0;
        if (!noAssert) checkOffset(offset, 1, this.length);
        return this[offset];
      };
      Buffer3.prototype.readUint16LE = Buffer3.prototype.readUInt16LE = function readUInt16LE(offset, noAssert) {
        offset = offset >>> 0;
        if (!noAssert) checkOffset(offset, 2, this.length);
        return this[offset] | this[offset + 1] << 8;
      };
      Buffer3.prototype.readUint16BE = Buffer3.prototype.readUInt16BE = function readUInt16BE(offset, noAssert) {
        offset = offset >>> 0;
        if (!noAssert) checkOffset(offset, 2, this.length);
        return this[offset] << 8 | this[offset + 1];
      };
      Buffer3.prototype.readUint32LE = Buffer3.prototype.readUInt32LE = function readUInt32LE(offset, noAssert) {
        offset = offset >>> 0;
        if (!noAssert) checkOffset(offset, 4, this.length);
        return (this[offset] | this[offset + 1] << 8 | this[offset + 2] << 16) + this[offset + 3] * 16777216;
      };
      Buffer3.prototype.readUint32BE = Buffer3.prototype.readUInt32BE = function readUInt32BE(offset, noAssert) {
        offset = offset >>> 0;
        if (!noAssert) checkOffset(offset, 4, this.length);
        return this[offset] * 16777216 + (this[offset + 1] << 16 | this[offset + 2] << 8 | this[offset + 3]);
      };
      Buffer3.prototype.readBigUInt64LE = defineBigIntMethod(function readBigUInt64LE(offset) {
        offset = offset >>> 0;
        validateNumber(offset, "offset");
        const first = this[offset];
        const last = this[offset + 7];
        if (first === void 0 || last === void 0) {
          boundsError(offset, this.length - 8);
        }
        const lo = first + this[++offset] * 2 ** 8 + this[++offset] * 2 ** 16 + this[++offset] * 2 ** 24;
        const hi = this[++offset] + this[++offset] * 2 ** 8 + this[++offset] * 2 ** 16 + last * 2 ** 24;
        return BigInt(lo) + (BigInt(hi) << BigInt(32));
      });
      Buffer3.prototype.readBigUInt64BE = defineBigIntMethod(function readBigUInt64BE(offset) {
        offset = offset >>> 0;
        validateNumber(offset, "offset");
        const first = this[offset];
        const last = this[offset + 7];
        if (first === void 0 || last === void 0) {
          boundsError(offset, this.length - 8);
        }
        const hi = first * 2 ** 24 + this[++offset] * 2 ** 16 + this[++offset] * 2 ** 8 + this[++offset];
        const lo = this[++offset] * 2 ** 24 + this[++offset] * 2 ** 16 + this[++offset] * 2 ** 8 + last;
        return (BigInt(hi) << BigInt(32)) + BigInt(lo);
      });
      Buffer3.prototype.readIntLE = function readIntLE(offset, byteLength2, noAssert) {
        offset = offset >>> 0;
        byteLength2 = byteLength2 >>> 0;
        if (!noAssert) checkOffset(offset, byteLength2, this.length);
        let val = this[offset];
        let mul = 1;
        let i = 0;
        while (++i < byteLength2 && (mul *= 256)) {
          val += this[offset + i] * mul;
        }
        mul *= 128;
        if (val >= mul) val -= Math.pow(2, 8 * byteLength2);
        return val;
      };
      Buffer3.prototype.readIntBE = function readIntBE(offset, byteLength2, noAssert) {
        offset = offset >>> 0;
        byteLength2 = byteLength2 >>> 0;
        if (!noAssert) checkOffset(offset, byteLength2, this.length);
        let i = byteLength2;
        let mul = 1;
        let val = this[offset + --i];
        while (i > 0 && (mul *= 256)) {
          val += this[offset + --i] * mul;
        }
        mul *= 128;
        if (val >= mul) val -= Math.pow(2, 8 * byteLength2);
        return val;
      };
      Buffer3.prototype.readInt8 = function readInt8(offset, noAssert) {
        offset = offset >>> 0;
        if (!noAssert) checkOffset(offset, 1, this.length);
        if (!(this[offset] & 128)) return this[offset];
        return (255 - this[offset] + 1) * -1;
      };
      Buffer3.prototype.readInt16LE = function readInt16LE(offset, noAssert) {
        offset = offset >>> 0;
        if (!noAssert) checkOffset(offset, 2, this.length);
        const val = this[offset] | this[offset + 1] << 8;
        return val & 32768 ? val | 4294901760 : val;
      };
      Buffer3.prototype.readInt16BE = function readInt16BE(offset, noAssert) {
        offset = offset >>> 0;
        if (!noAssert) checkOffset(offset, 2, this.length);
        const val = this[offset + 1] | this[offset] << 8;
        return val & 32768 ? val | 4294901760 : val;
      };
      Buffer3.prototype.readInt32LE = function readInt32LE(offset, noAssert) {
        offset = offset >>> 0;
        if (!noAssert) checkOffset(offset, 4, this.length);
        return this[offset] | this[offset + 1] << 8 | this[offset + 2] << 16 | this[offset + 3] << 24;
      };
      Buffer3.prototype.readInt32BE = function readInt32BE(offset, noAssert) {
        offset = offset >>> 0;
        if (!noAssert) checkOffset(offset, 4, this.length);
        return this[offset] << 24 | this[offset + 1] << 16 | this[offset + 2] << 8 | this[offset + 3];
      };
      Buffer3.prototype.readBigInt64LE = defineBigIntMethod(function readBigInt64LE(offset) {
        offset = offset >>> 0;
        validateNumber(offset, "offset");
        const first = this[offset];
        const last = this[offset + 7];
        if (first === void 0 || last === void 0) {
          boundsError(offset, this.length - 8);
        }
        const val = this[offset + 4] + this[offset + 5] * 2 ** 8 + this[offset + 6] * 2 ** 16 + (last << 24);
        return (BigInt(val) << BigInt(32)) + BigInt(first + this[++offset] * 2 ** 8 + this[++offset] * 2 ** 16 + this[++offset] * 2 ** 24);
      });
      Buffer3.prototype.readBigInt64BE = defineBigIntMethod(function readBigInt64BE(offset) {
        offset = offset >>> 0;
        validateNumber(offset, "offset");
        const first = this[offset];
        const last = this[offset + 7];
        if (first === void 0 || last === void 0) {
          boundsError(offset, this.length - 8);
        }
        const val = (first << 24) + // Overflow
        this[++offset] * 2 ** 16 + this[++offset] * 2 ** 8 + this[++offset];
        return (BigInt(val) << BigInt(32)) + BigInt(this[++offset] * 2 ** 24 + this[++offset] * 2 ** 16 + this[++offset] * 2 ** 8 + last);
      });
      Buffer3.prototype.readFloatLE = function readFloatLE(offset, noAssert) {
        offset = offset >>> 0;
        if (!noAssert) checkOffset(offset, 4, this.length);
        return ieee754.read(this, offset, true, 23, 4);
      };
      Buffer3.prototype.readFloatBE = function readFloatBE(offset, noAssert) {
        offset = offset >>> 0;
        if (!noAssert) checkOffset(offset, 4, this.length);
        return ieee754.read(this, offset, false, 23, 4);
      };
      Buffer3.prototype.readDoubleLE = function readDoubleLE(offset, noAssert) {
        offset = offset >>> 0;
        if (!noAssert) checkOffset(offset, 8, this.length);
        return ieee754.read(this, offset, true, 52, 8);
      };
      Buffer3.prototype.readDoubleBE = function readDoubleBE(offset, noAssert) {
        offset = offset >>> 0;
        if (!noAssert) checkOffset(offset, 8, this.length);
        return ieee754.read(this, offset, false, 52, 8);
      };
      function checkInt(buf, value, offset, ext, max, min) {
        if (!Buffer3.isBuffer(buf)) throw new TypeError('"buffer" argument must be a Buffer instance');
        if (value > max || value < min) throw new RangeError('"value" argument is out of bounds');
        if (offset + ext > buf.length) throw new RangeError("Index out of range");
      }
      Buffer3.prototype.writeUintLE = Buffer3.prototype.writeUIntLE = function writeUIntLE(value, offset, byteLength2, noAssert) {
        value = +value;
        offset = offset >>> 0;
        byteLength2 = byteLength2 >>> 0;
        if (!noAssert) {
          const maxBytes = Math.pow(2, 8 * byteLength2) - 1;
          checkInt(this, value, offset, byteLength2, maxBytes, 0);
        }
        let mul = 1;
        let i = 0;
        this[offset] = value & 255;
        while (++i < byteLength2 && (mul *= 256)) {
          this[offset + i] = value / mul & 255;
        }
        return offset + byteLength2;
      };
      Buffer3.prototype.writeUintBE = Buffer3.prototype.writeUIntBE = function writeUIntBE(value, offset, byteLength2, noAssert) {
        value = +value;
        offset = offset >>> 0;
        byteLength2 = byteLength2 >>> 0;
        if (!noAssert) {
          const maxBytes = Math.pow(2, 8 * byteLength2) - 1;
          checkInt(this, value, offset, byteLength2, maxBytes, 0);
        }
        let i = byteLength2 - 1;
        let mul = 1;
        this[offset + i] = value & 255;
        while (--i >= 0 && (mul *= 256)) {
          this[offset + i] = value / mul & 255;
        }
        return offset + byteLength2;
      };
      Buffer3.prototype.writeUint8 = Buffer3.prototype.writeUInt8 = function writeUInt8(value, offset, noAssert) {
        value = +value;
        offset = offset >>> 0;
        if (!noAssert) checkInt(this, value, offset, 1, 255, 0);
        this[offset] = value & 255;
        return offset + 1;
      };
      Buffer3.prototype.writeUint16LE = Buffer3.prototype.writeUInt16LE = function writeUInt16LE(value, offset, noAssert) {
        value = +value;
        offset = offset >>> 0;
        if (!noAssert) checkInt(this, value, offset, 2, 65535, 0);
        this[offset] = value & 255;
        this[offset + 1] = value >>> 8;
        return offset + 2;
      };
      Buffer3.prototype.writeUint16BE = Buffer3.prototype.writeUInt16BE = function writeUInt16BE(value, offset, noAssert) {
        value = +value;
        offset = offset >>> 0;
        if (!noAssert) checkInt(this, value, offset, 2, 65535, 0);
        this[offset] = value >>> 8;
        this[offset + 1] = value & 255;
        return offset + 2;
      };
      Buffer3.prototype.writeUint32LE = Buffer3.prototype.writeUInt32LE = function writeUInt32LE(value, offset, noAssert) {
        value = +value;
        offset = offset >>> 0;
        if (!noAssert) checkInt(this, value, offset, 4, 4294967295, 0);
        this[offset + 3] = value >>> 24;
        this[offset + 2] = value >>> 16;
        this[offset + 1] = value >>> 8;
        this[offset] = value & 255;
        return offset + 4;
      };
      Buffer3.prototype.writeUint32BE = Buffer3.prototype.writeUInt32BE = function writeUInt32BE(value, offset, noAssert) {
        value = +value;
        offset = offset >>> 0;
        if (!noAssert) checkInt(this, value, offset, 4, 4294967295, 0);
        this[offset] = value >>> 24;
        this[offset + 1] = value >>> 16;
        this[offset + 2] = value >>> 8;
        this[offset + 3] = value & 255;
        return offset + 4;
      };
      function wrtBigUInt64LE(buf, value, offset, min, max) {
        checkIntBI(value, min, max, buf, offset, 7);
        let lo = Number(value & BigInt(4294967295));
        buf[offset++] = lo;
        lo = lo >> 8;
        buf[offset++] = lo;
        lo = lo >> 8;
        buf[offset++] = lo;
        lo = lo >> 8;
        buf[offset++] = lo;
        let hi = Number(value >> BigInt(32) & BigInt(4294967295));
        buf[offset++] = hi;
        hi = hi >> 8;
        buf[offset++] = hi;
        hi = hi >> 8;
        buf[offset++] = hi;
        hi = hi >> 8;
        buf[offset++] = hi;
        return offset;
      }
      function wrtBigUInt64BE(buf, value, offset, min, max) {
        checkIntBI(value, min, max, buf, offset, 7);
        let lo = Number(value & BigInt(4294967295));
        buf[offset + 7] = lo;
        lo = lo >> 8;
        buf[offset + 6] = lo;
        lo = lo >> 8;
        buf[offset + 5] = lo;
        lo = lo >> 8;
        buf[offset + 4] = lo;
        let hi = Number(value >> BigInt(32) & BigInt(4294967295));
        buf[offset + 3] = hi;
        hi = hi >> 8;
        buf[offset + 2] = hi;
        hi = hi >> 8;
        buf[offset + 1] = hi;
        hi = hi >> 8;
        buf[offset] = hi;
        return offset + 8;
      }
      Buffer3.prototype.writeBigUInt64LE = defineBigIntMethod(function writeBigUInt64LE(value, offset = 0) {
        return wrtBigUInt64LE(this, value, offset, BigInt(0), BigInt("0xffffffffffffffff"));
      });
      Buffer3.prototype.writeBigUInt64BE = defineBigIntMethod(function writeBigUInt64BE(value, offset = 0) {
        return wrtBigUInt64BE(this, value, offset, BigInt(0), BigInt("0xffffffffffffffff"));
      });
      Buffer3.prototype.writeIntLE = function writeIntLE(value, offset, byteLength2, noAssert) {
        value = +value;
        offset = offset >>> 0;
        if (!noAssert) {
          const limit = Math.pow(2, 8 * byteLength2 - 1);
          checkInt(this, value, offset, byteLength2, limit - 1, -limit);
        }
        let i = 0;
        let mul = 1;
        let sub = 0;
        this[offset] = value & 255;
        while (++i < byteLength2 && (mul *= 256)) {
          if (value < 0 && sub === 0 && this[offset + i - 1] !== 0) {
            sub = 1;
          }
          this[offset + i] = (value / mul >> 0) - sub & 255;
        }
        return offset + byteLength2;
      };
      Buffer3.prototype.writeIntBE = function writeIntBE(value, offset, byteLength2, noAssert) {
        value = +value;
        offset = offset >>> 0;
        if (!noAssert) {
          const limit = Math.pow(2, 8 * byteLength2 - 1);
          checkInt(this, value, offset, byteLength2, limit - 1, -limit);
        }
        let i = byteLength2 - 1;
        let mul = 1;
        let sub = 0;
        this[offset + i] = value & 255;
        while (--i >= 0 && (mul *= 256)) {
          if (value < 0 && sub === 0 && this[offset + i + 1] !== 0) {
            sub = 1;
          }
          this[offset + i] = (value / mul >> 0) - sub & 255;
        }
        return offset + byteLength2;
      };
      Buffer3.prototype.writeInt8 = function writeInt8(value, offset, noAssert) {
        value = +value;
        offset = offset >>> 0;
        if (!noAssert) checkInt(this, value, offset, 1, 127, -128);
        if (value < 0) value = 255 + value + 1;
        this[offset] = value & 255;
        return offset + 1;
      };
      Buffer3.prototype.writeInt16LE = function writeInt16LE(value, offset, noAssert) {
        value = +value;
        offset = offset >>> 0;
        if (!noAssert) checkInt(this, value, offset, 2, 32767, -32768);
        this[offset] = value & 255;
        this[offset + 1] = value >>> 8;
        return offset + 2;
      };
      Buffer3.prototype.writeInt16BE = function writeInt16BE(value, offset, noAssert) {
        value = +value;
        offset = offset >>> 0;
        if (!noAssert) checkInt(this, value, offset, 2, 32767, -32768);
        this[offset] = value >>> 8;
        this[offset + 1] = value & 255;
        return offset + 2;
      };
      Buffer3.prototype.writeInt32LE = function writeInt32LE(value, offset, noAssert) {
        value = +value;
        offset = offset >>> 0;
        if (!noAssert) checkInt(this, value, offset, 4, 2147483647, -2147483648);
        this[offset] = value & 255;
        this[offset + 1] = value >>> 8;
        this[offset + 2] = value >>> 16;
        this[offset + 3] = value >>> 24;
        return offset + 4;
      };
      Buffer3.prototype.writeInt32BE = function writeInt32BE(value, offset, noAssert) {
        value = +value;
        offset = offset >>> 0;
        if (!noAssert) checkInt(this, value, offset, 4, 2147483647, -2147483648);
        if (value < 0) value = 4294967295 + value + 1;
        this[offset] = value >>> 24;
        this[offset + 1] = value >>> 16;
        this[offset + 2] = value >>> 8;
        this[offset + 3] = value & 255;
        return offset + 4;
      };
      Buffer3.prototype.writeBigInt64LE = defineBigIntMethod(function writeBigInt64LE(value, offset = 0) {
        return wrtBigUInt64LE(this, value, offset, -BigInt("0x8000000000000000"), BigInt("0x7fffffffffffffff"));
      });
      Buffer3.prototype.writeBigInt64BE = defineBigIntMethod(function writeBigInt64BE(value, offset = 0) {
        return wrtBigUInt64BE(this, value, offset, -BigInt("0x8000000000000000"), BigInt("0x7fffffffffffffff"));
      });
      function checkIEEE754(buf, value, offset, ext, max, min) {
        if (offset + ext > buf.length) throw new RangeError("Index out of range");
        if (offset < 0) throw new RangeError("Index out of range");
      }
      function writeFloat(buf, value, offset, littleEndian, noAssert) {
        value = +value;
        offset = offset >>> 0;
        if (!noAssert) {
          checkIEEE754(buf, value, offset, 4, 34028234663852886e22, -34028234663852886e22);
        }
        ieee754.write(buf, value, offset, littleEndian, 23, 4);
        return offset + 4;
      }
      Buffer3.prototype.writeFloatLE = function writeFloatLE(value, offset, noAssert) {
        return writeFloat(this, value, offset, true, noAssert);
      };
      Buffer3.prototype.writeFloatBE = function writeFloatBE(value, offset, noAssert) {
        return writeFloat(this, value, offset, false, noAssert);
      };
      function writeDouble(buf, value, offset, littleEndian, noAssert) {
        value = +value;
        offset = offset >>> 0;
        if (!noAssert) {
          checkIEEE754(buf, value, offset, 8, 17976931348623157e292, -17976931348623157e292);
        }
        ieee754.write(buf, value, offset, littleEndian, 52, 8);
        return offset + 8;
      }
      Buffer3.prototype.writeDoubleLE = function writeDoubleLE(value, offset, noAssert) {
        return writeDouble(this, value, offset, true, noAssert);
      };
      Buffer3.prototype.writeDoubleBE = function writeDoubleBE(value, offset, noAssert) {
        return writeDouble(this, value, offset, false, noAssert);
      };
      Buffer3.prototype.copy = function copy(target, targetStart, start, end) {
        if (!Buffer3.isBuffer(target)) throw new TypeError("argument should be a Buffer");
        if (!start) start = 0;
        if (!end && end !== 0) end = this.length;
        if (targetStart >= target.length) targetStart = target.length;
        if (!targetStart) targetStart = 0;
        if (end > 0 && end < start) end = start;
        if (end === start) return 0;
        if (target.length === 0 || this.length === 0) return 0;
        if (targetStart < 0) {
          throw new RangeError("targetStart out of bounds");
        }
        if (start < 0 || start >= this.length) throw new RangeError("Index out of range");
        if (end < 0) throw new RangeError("sourceEnd out of bounds");
        if (end > this.length) end = this.length;
        if (target.length - targetStart < end - start) {
          end = target.length - targetStart + start;
        }
        const len = end - start;
        if (this === target && typeof Uint8Array.prototype.copyWithin === "function") {
          this.copyWithin(targetStart, start, end);
        } else {
          Uint8Array.prototype.set.call(
            target,
            this.subarray(start, end),
            targetStart
          );
        }
        return len;
      };
      Buffer3.prototype.fill = function fill(val, start, end, encoding) {
        if (typeof val === "string") {
          if (typeof start === "string") {
            encoding = start;
            start = 0;
            end = this.length;
          } else if (typeof end === "string") {
            encoding = end;
            end = this.length;
          }
          if (encoding !== void 0 && typeof encoding !== "string") {
            throw new TypeError("encoding must be a string");
          }
          if (typeof encoding === "string" && !Buffer3.isEncoding(encoding)) {
            throw new TypeError("Unknown encoding: " + encoding);
          }
          if (val.length === 1) {
            const code = val.charCodeAt(0);
            if (encoding === "utf8" && code < 128 || encoding === "latin1") {
              val = code;
            }
          }
        } else if (typeof val === "number") {
          val = val & 255;
        } else if (typeof val === "boolean") {
          val = Number(val);
        }
        if (start < 0 || this.length < start || this.length < end) {
          throw new RangeError("Out of range index");
        }
        if (end <= start) {
          return this;
        }
        start = start >>> 0;
        end = end === void 0 ? this.length : end >>> 0;
        if (!val) val = 0;
        let i;
        if (typeof val === "number") {
          for (i = start; i < end; ++i) {
            this[i] = val;
          }
        } else {
          const bytes = Buffer3.isBuffer(val) ? val : Buffer3.from(val, encoding);
          const len = bytes.length;
          if (len === 0) {
            throw new TypeError('The value "' + val + '" is invalid for argument "value"');
          }
          for (i = 0; i < end - start; ++i) {
            this[i + start] = bytes[i % len];
          }
        }
        return this;
      };
      var errors = {};
      function E(sym, getMessage, Base) {
        errors[sym] = class NodeError extends Base {
          constructor() {
            super();
            Object.defineProperty(this, "message", {
              value: getMessage.apply(this, arguments),
              writable: true,
              configurable: true
            });
            this.name = `${this.name} [${sym}]`;
            this.stack;
            delete this.name;
          }
          get code() {
            return sym;
          }
          set code(value) {
            Object.defineProperty(this, "code", {
              configurable: true,
              enumerable: true,
              value,
              writable: true
            });
          }
          toString() {
            return `${this.name} [${sym}]: ${this.message}`;
          }
        };
      }
      E(
        "ERR_BUFFER_OUT_OF_BOUNDS",
        function(name) {
          if (name) {
            return `${name} is outside of buffer bounds`;
          }
          return "Attempt to access memory outside buffer bounds";
        },
        RangeError
      );
      E(
        "ERR_INVALID_ARG_TYPE",
        function(name, actual) {
          return `The "${name}" argument must be of type number. Received type ${typeof actual}`;
        },
        TypeError
      );
      E(
        "ERR_OUT_OF_RANGE",
        function(str, range, input) {
          let msg = `The value of "${str}" is out of range.`;
          let received = input;
          if (Number.isInteger(input) && Math.abs(input) > 2 ** 32) {
            received = addNumericalSeparator(String(input));
          } else if (typeof input === "bigint") {
            received = String(input);
            if (input > BigInt(2) ** BigInt(32) || input < -(BigInt(2) ** BigInt(32))) {
              received = addNumericalSeparator(received);
            }
            received += "n";
          }
          msg += ` It must be ${range}. Received ${received}`;
          return msg;
        },
        RangeError
      );
      function addNumericalSeparator(val) {
        let res = "";
        let i = val.length;
        const start = val[0] === "-" ? 1 : 0;
        for (; i >= start + 4; i -= 3) {
          res = `_${val.slice(i - 3, i)}${res}`;
        }
        return `${val.slice(0, i)}${res}`;
      }
      function checkBounds(buf, offset, byteLength2) {
        validateNumber(offset, "offset");
        if (buf[offset] === void 0 || buf[offset + byteLength2] === void 0) {
          boundsError(offset, buf.length - (byteLength2 + 1));
        }
      }
      function checkIntBI(value, min, max, buf, offset, byteLength2) {
        if (value > max || value < min) {
          const n = typeof min === "bigint" ? "n" : "";
          let range;
          if (byteLength2 > 3) {
            if (min === 0 || min === BigInt(0)) {
              range = `>= 0${n} and < 2${n} ** ${(byteLength2 + 1) * 8}${n}`;
            } else {
              range = `>= -(2${n} ** ${(byteLength2 + 1) * 8 - 1}${n}) and < 2 ** ${(byteLength2 + 1) * 8 - 1}${n}`;
            }
          } else {
            range = `>= ${min}${n} and <= ${max}${n}`;
          }
          throw new errors.ERR_OUT_OF_RANGE("value", range, value);
        }
        checkBounds(buf, offset, byteLength2);
      }
      function validateNumber(value, name) {
        if (typeof value !== "number") {
          throw new errors.ERR_INVALID_ARG_TYPE(name, "number", value);
        }
      }
      function boundsError(value, length, type) {
        if (Math.floor(value) !== value) {
          validateNumber(value, type);
          throw new errors.ERR_OUT_OF_RANGE(type || "offset", "an integer", value);
        }
        if (length < 0) {
          throw new errors.ERR_BUFFER_OUT_OF_BOUNDS();
        }
        throw new errors.ERR_OUT_OF_RANGE(
          type || "offset",
          `>= ${type ? 1 : 0} and <= ${length}`,
          value
        );
      }
      var INVALID_BASE64_RE = /[^+/0-9A-Za-z-_]/g;
      function base64clean(str) {
        str = str.split("=")[0];
        str = str.trim().replace(INVALID_BASE64_RE, "");
        if (str.length < 2) return "";
        while (str.length % 4 !== 0) {
          str = str + "=";
        }
        return str;
      }
      function utf8ToBytes(string, units) {
        units = units || Infinity;
        let codePoint;
        const length = string.length;
        let leadSurrogate = null;
        const bytes = [];
        for (let i = 0; i < length; ++i) {
          codePoint = string.charCodeAt(i);
          if (codePoint > 55295 && codePoint < 57344) {
            if (!leadSurrogate) {
              if (codePoint > 56319) {
                if ((units -= 3) > -1) bytes.push(239, 191, 189);
                continue;
              } else if (i + 1 === length) {
                if ((units -= 3) > -1) bytes.push(239, 191, 189);
                continue;
              }
              leadSurrogate = codePoint;
              continue;
            }
            if (codePoint < 56320) {
              if ((units -= 3) > -1) bytes.push(239, 191, 189);
              leadSurrogate = codePoint;
              continue;
            }
            codePoint = (leadSurrogate - 55296 << 10 | codePoint - 56320) + 65536;
          } else if (leadSurrogate) {
            if ((units -= 3) > -1) bytes.push(239, 191, 189);
          }
          leadSurrogate = null;
          if (codePoint < 128) {
            if ((units -= 1) < 0) break;
            bytes.push(codePoint);
          } else if (codePoint < 2048) {
            if ((units -= 2) < 0) break;
            bytes.push(
              codePoint >> 6 | 192,
              codePoint & 63 | 128
            );
          } else if (codePoint < 65536) {
            if ((units -= 3) < 0) break;
            bytes.push(
              codePoint >> 12 | 224,
              codePoint >> 6 & 63 | 128,
              codePoint & 63 | 128
            );
          } else if (codePoint < 1114112) {
            if ((units -= 4) < 0) break;
            bytes.push(
              codePoint >> 18 | 240,
              codePoint >> 12 & 63 | 128,
              codePoint >> 6 & 63 | 128,
              codePoint & 63 | 128
            );
          } else {
            throw new Error("Invalid code point");
          }
        }
        return bytes;
      }
      function asciiToBytes(str) {
        const byteArray = [];
        for (let i = 0; i < str.length; ++i) {
          byteArray.push(str.charCodeAt(i) & 255);
        }
        return byteArray;
      }
      function utf16leToBytes(str, units) {
        let c, hi, lo;
        const byteArray = [];
        for (let i = 0; i < str.length; ++i) {
          if ((units -= 2) < 0) break;
          c = str.charCodeAt(i);
          hi = c >> 8;
          lo = c % 256;
          byteArray.push(lo);
          byteArray.push(hi);
        }
        return byteArray;
      }
      function base64ToBytes(str) {
        return base64.toByteArray(base64clean(str));
      }
      function blitBuffer(src, dst, offset, length) {
        let i;
        for (i = 0; i < length; ++i) {
          if (i + offset >= dst.length || i >= src.length) break;
          dst[i + offset] = src[i];
        }
        return i;
      }
      function isInstance(obj, type) {
        return obj instanceof type || obj != null && obj.constructor != null && obj.constructor.name != null && obj.constructor.name === type.name;
      }
      function numberIsNaN(obj) {
        return obj !== obj;
      }
      var hexSliceLookupTable = (function() {
        const alphabet = "0123456789abcdef";
        const table = new Array(256);
        for (let i = 0; i < 16; ++i) {
          const i16 = i * 16;
          for (let j = 0; j < 16; ++j) {
            table[i16 + j] = alphabet[i] + alphabet[j];
          }
        }
        return table;
      })();
      function defineBigIntMethod(fn) {
        return typeof BigInt === "undefined" ? BufferBigIntNotDefined : fn;
      }
      function BufferBigIntNotDefined() {
        throw new Error("BigInt not supported");
      }
    }
  });

  // node_modules/ms/index.js
  var require_ms = __commonJS({
    "node_modules/ms/index.js"(exports, module) {
      var s = 1e3;
      var m = s * 60;
      var h = m * 60;
      var d = h * 24;
      var w = d * 7;
      var y = d * 365.25;
      module.exports = function(val, options) {
        options = options || {};
        var type = typeof val;
        if (type === "string" && val.length > 0) {
          return parse(val);
        } else if (type === "number" && isFinite(val)) {
          return options.long ? fmtLong(val) : fmtShort(val);
        }
        throw new Error(
          "val is not a non-empty string or a valid number. val=" + JSON.stringify(val)
        );
      };
      function parse(str) {
        str = String(str);
        if (str.length > 100) {
          return;
        }
        var match = /^(-?(?:\d+)?\.?\d+) *(milliseconds?|msecs?|ms|seconds?|secs?|s|minutes?|mins?|m|hours?|hrs?|h|days?|d|weeks?|w|years?|yrs?|y)?$/i.exec(
          str
        );
        if (!match) {
          return;
        }
        var n = parseFloat(match[1]);
        var type = (match[2] || "ms").toLowerCase();
        switch (type) {
          case "years":
          case "year":
          case "yrs":
          case "yr":
          case "y":
            return n * y;
          case "weeks":
          case "week":
          case "w":
            return n * w;
          case "days":
          case "day":
          case "d":
            return n * d;
          case "hours":
          case "hour":
          case "hrs":
          case "hr":
          case "h":
            return n * h;
          case "minutes":
          case "minute":
          case "mins":
          case "min":
          case "m":
            return n * m;
          case "seconds":
          case "second":
          case "secs":
          case "sec":
          case "s":
            return n * s;
          case "milliseconds":
          case "millisecond":
          case "msecs":
          case "msec":
          case "ms":
            return n;
          default:
            return void 0;
        }
      }
      function fmtShort(ms) {
        var msAbs = Math.abs(ms);
        if (msAbs >= d) {
          return Math.round(ms / d) + "d";
        }
        if (msAbs >= h) {
          return Math.round(ms / h) + "h";
        }
        if (msAbs >= m) {
          return Math.round(ms / m) + "m";
        }
        if (msAbs >= s) {
          return Math.round(ms / s) + "s";
        }
        return ms + "ms";
      }
      function fmtLong(ms) {
        var msAbs = Math.abs(ms);
        if (msAbs >= d) {
          return plural(ms, msAbs, d, "day");
        }
        if (msAbs >= h) {
          return plural(ms, msAbs, h, "hour");
        }
        if (msAbs >= m) {
          return plural(ms, msAbs, m, "minute");
        }
        if (msAbs >= s) {
          return plural(ms, msAbs, s, "second");
        }
        return ms + " ms";
      }
      function plural(ms, msAbs, n, name) {
        var isPlural = msAbs >= n * 1.5;
        return Math.round(ms / n) + " " + name + (isPlural ? "s" : "");
      }
    }
  });

  // node_modules/debug/src/common.js
  var require_common = __commonJS({
    "node_modules/debug/src/common.js"(exports, module) {
      function setup(env) {
        createDebug.debug = createDebug;
        createDebug.default = createDebug;
        createDebug.coerce = coerce;
        createDebug.disable = disable;
        createDebug.enable = enable;
        createDebug.enabled = enabled;
        createDebug.humanize = require_ms();
        createDebug.destroy = destroy;
        Object.keys(env).forEach((key) => {
          createDebug[key] = env[key];
        });
        createDebug.names = [];
        createDebug.skips = [];
        createDebug.formatters = {};
        function selectColor(namespace) {
          let hash = 0;
          for (let i = 0; i < namespace.length; i++) {
            hash = (hash << 5) - hash + namespace.charCodeAt(i);
            hash |= 0;
          }
          return createDebug.colors[Math.abs(hash) % createDebug.colors.length];
        }
        createDebug.selectColor = selectColor;
        function createDebug(namespace) {
          let prevTime;
          let enableOverride = null;
          let namespacesCache;
          let enabledCache;
          function debug15(...args) {
            if (!debug15.enabled) {
              return;
            }
            const self = debug15;
            const curr = Number(/* @__PURE__ */ new Date());
            const ms = curr - (prevTime || curr);
            self.diff = ms;
            self.prev = prevTime;
            self.curr = curr;
            prevTime = curr;
            args[0] = createDebug.coerce(args[0]);
            if (typeof args[0] !== "string") {
              args.unshift("%O");
            }
            let index = 0;
            args[0] = args[0].replace(/%([a-zA-Z%])/g, (match, format) => {
              if (match === "%%") {
                return "%";
              }
              index++;
              const formatter = createDebug.formatters[format];
              if (typeof formatter === "function") {
                const val = args[index];
                match = formatter.call(self, val);
                args.splice(index, 1);
                index--;
              }
              return match;
            });
            createDebug.formatArgs.call(self, args);
            const logFn = self.log || createDebug.log;
            logFn.apply(self, args);
          }
          debug15.namespace = namespace;
          debug15.useColors = createDebug.useColors();
          debug15.color = createDebug.selectColor(namespace);
          debug15.extend = extend;
          debug15.destroy = createDebug.destroy;
          Object.defineProperty(debug15, "enabled", {
            enumerable: true,
            configurable: false,
            get: () => {
              if (enableOverride !== null) {
                return enableOverride;
              }
              if (namespacesCache !== createDebug.namespaces) {
                namespacesCache = createDebug.namespaces;
                enabledCache = createDebug.enabled(namespace);
              }
              return enabledCache;
            },
            set: (v) => {
              enableOverride = v;
            }
          });
          if (typeof createDebug.init === "function") {
            createDebug.init(debug15);
          }
          return debug15;
        }
        function extend(namespace, delimiter) {
          const newDebug = createDebug(this.namespace + (typeof delimiter === "undefined" ? ":" : delimiter) + namespace);
          newDebug.log = this.log;
          return newDebug;
        }
        function enable(namespaces) {
          createDebug.save(namespaces);
          createDebug.namespaces = namespaces;
          createDebug.names = [];
          createDebug.skips = [];
          const split = (typeof namespaces === "string" ? namespaces : "").trim().replace(/\s+/g, ",").split(",").filter(Boolean);
          for (const ns of split) {
            if (ns[0] === "-") {
              createDebug.skips.push(ns.slice(1));
            } else {
              createDebug.names.push(ns);
            }
          }
        }
        function matchesTemplate(search, template) {
          let searchIndex = 0;
          let templateIndex = 0;
          let starIndex = -1;
          let matchIndex = 0;
          while (searchIndex < search.length) {
            if (templateIndex < template.length && (template[templateIndex] === search[searchIndex] || template[templateIndex] === "*")) {
              if (template[templateIndex] === "*") {
                starIndex = templateIndex;
                matchIndex = searchIndex;
                templateIndex++;
              } else {
                searchIndex++;
                templateIndex++;
              }
            } else if (starIndex !== -1) {
              templateIndex = starIndex + 1;
              matchIndex++;
              searchIndex = matchIndex;
            } else {
              return false;
            }
          }
          while (templateIndex < template.length && template[templateIndex] === "*") {
            templateIndex++;
          }
          return templateIndex === template.length;
        }
        function disable() {
          const namespaces = [
            ...createDebug.names,
            ...createDebug.skips.map((namespace) => "-" + namespace)
          ].join(",");
          createDebug.enable("");
          return namespaces;
        }
        function enabled(name) {
          for (const skip of createDebug.skips) {
            if (matchesTemplate(name, skip)) {
              return false;
            }
          }
          for (const ns of createDebug.names) {
            if (matchesTemplate(name, ns)) {
              return true;
            }
          }
          return false;
        }
        function coerce(val) {
          if (val instanceof Error) {
            return val.stack || val.message;
          }
          return val;
        }
        function destroy() {
          console.warn("Instance method `debug.destroy()` is deprecated and no longer does anything. It will be removed in the next major version of `debug`.");
        }
        createDebug.enable(createDebug.load());
        return createDebug;
      }
      module.exports = setup;
    }
  });

  // node_modules/debug/src/browser.js
  var require_browser = __commonJS({
    "node_modules/debug/src/browser.js"(exports, module) {
      exports.formatArgs = formatArgs;
      exports.save = save;
      exports.load = load;
      exports.useColors = useColors;
      exports.storage = localstorage();
      exports.destroy = /* @__PURE__ */ (() => {
        let warned = false;
        return () => {
          if (!warned) {
            warned = true;
            console.warn("Instance method `debug.destroy()` is deprecated and no longer does anything. It will be removed in the next major version of `debug`.");
          }
        };
      })();
      exports.colors = [
        "#0000CC",
        "#0000FF",
        "#0033CC",
        "#0033FF",
        "#0066CC",
        "#0066FF",
        "#0099CC",
        "#0099FF",
        "#00CC00",
        "#00CC33",
        "#00CC66",
        "#00CC99",
        "#00CCCC",
        "#00CCFF",
        "#3300CC",
        "#3300FF",
        "#3333CC",
        "#3333FF",
        "#3366CC",
        "#3366FF",
        "#3399CC",
        "#3399FF",
        "#33CC00",
        "#33CC33",
        "#33CC66",
        "#33CC99",
        "#33CCCC",
        "#33CCFF",
        "#6600CC",
        "#6600FF",
        "#6633CC",
        "#6633FF",
        "#66CC00",
        "#66CC33",
        "#9900CC",
        "#9900FF",
        "#9933CC",
        "#9933FF",
        "#99CC00",
        "#99CC33",
        "#CC0000",
        "#CC0033",
        "#CC0066",
        "#CC0099",
        "#CC00CC",
        "#CC00FF",
        "#CC3300",
        "#CC3333",
        "#CC3366",
        "#CC3399",
        "#CC33CC",
        "#CC33FF",
        "#CC6600",
        "#CC6633",
        "#CC9900",
        "#CC9933",
        "#CCCC00",
        "#CCCC33",
        "#FF0000",
        "#FF0033",
        "#FF0066",
        "#FF0099",
        "#FF00CC",
        "#FF00FF",
        "#FF3300",
        "#FF3333",
        "#FF3366",
        "#FF3399",
        "#FF33CC",
        "#FF33FF",
        "#FF6600",
        "#FF6633",
        "#FF9900",
        "#FF9933",
        "#FFCC00",
        "#FFCC33"
      ];
      function useColors() {
        if (typeof window !== "undefined" && window.process && (window.process.type === "renderer" || window.process.__nwjs)) {
          return true;
        }
        if (typeof navigator !== "undefined" && navigator.userAgent && navigator.userAgent.toLowerCase().match(/(edge|trident)\/(\d+)/)) {
          return false;
        }
        let m;
        return typeof document !== "undefined" && document.documentElement && document.documentElement.style && document.documentElement.style.WebkitAppearance || // Is firebug? http://stackoverflow.com/a/398120/376773
        typeof window !== "undefined" && window.console && (window.console.firebug || window.console.exception && window.console.table) || // Is firefox >= v31?
        // https://developer.mozilla.org/en-US/docs/Tools/Web_Console#Styling_messages
        typeof navigator !== "undefined" && navigator.userAgent && (m = navigator.userAgent.toLowerCase().match(/firefox\/(\d+)/)) && parseInt(m[1], 10) >= 31 || // Double check webkit in userAgent just in case we are in a worker
        typeof navigator !== "undefined" && navigator.userAgent && navigator.userAgent.toLowerCase().match(/applewebkit\/(\d+)/);
      }
      function formatArgs(args) {
        args[0] = (this.useColors ? "%c" : "") + this.namespace + (this.useColors ? " %c" : " ") + args[0] + (this.useColors ? "%c " : " ") + "+" + module.exports.humanize(this.diff);
        if (!this.useColors) {
          return;
        }
        const c = "color: " + this.color;
        args.splice(1, 0, c, "color: inherit");
        let index = 0;
        let lastC = 0;
        args[0].replace(/%[a-zA-Z%]/g, (match) => {
          if (match === "%%") {
            return;
          }
          index++;
          if (match === "%c") {
            lastC = index;
          }
        });
        args.splice(lastC, 0, c);
      }
      exports.log = console.debug || console.log || (() => {
      });
      function save(namespaces) {
        try {
          if (namespaces) {
            exports.storage.setItem("debug", namespaces);
          } else {
            exports.storage.removeItem("debug");
          }
        } catch (error) {
        }
      }
      function load() {
        let r;
        try {
          r = exports.storage.getItem("debug") || exports.storage.getItem("DEBUG");
        } catch (error) {
        }
        if (!r && typeof process !== "undefined" && "env" in process) {
          r = process.env.DEBUG;
        }
        return r;
      }
      function localstorage() {
        try {
          return localStorage;
        } catch (error) {
        }
      }
      module.exports = require_common()(exports);
      var { formatters } = module.exports;
      formatters.j = function(v) {
        try {
          return JSON.stringify(v);
        } catch (error) {
          return "[UnexpectedJSONParseError]: " + error.message;
        }
      };
    }
  });

  // node_modules/events/events.js
  var require_events = __commonJS({
    "node_modules/events/events.js"(exports, module) {
      "use strict";
      var R = typeof Reflect === "object" ? Reflect : null;
      var ReflectApply = R && typeof R.apply === "function" ? R.apply : function ReflectApply2(target, receiver, args) {
        return Function.prototype.apply.call(target, receiver, args);
      };
      var ReflectOwnKeys;
      if (R && typeof R.ownKeys === "function") {
        ReflectOwnKeys = R.ownKeys;
      } else if (Object.getOwnPropertySymbols) {
        ReflectOwnKeys = function ReflectOwnKeys2(target) {
          return Object.getOwnPropertyNames(target).concat(Object.getOwnPropertySymbols(target));
        };
      } else {
        ReflectOwnKeys = function ReflectOwnKeys2(target) {
          return Object.getOwnPropertyNames(target);
        };
      }
      function ProcessEmitWarning(warning) {
        if (console && console.warn) console.warn(warning);
      }
      var NumberIsNaN = Number.isNaN || function NumberIsNaN2(value) {
        return value !== value;
      };
      function EventEmitter5() {
        EventEmitter5.init.call(this);
      }
      module.exports = EventEmitter5;
      module.exports.once = once;
      EventEmitter5.EventEmitter = EventEmitter5;
      EventEmitter5.prototype._events = void 0;
      EventEmitter5.prototype._eventsCount = 0;
      EventEmitter5.prototype._maxListeners = void 0;
      var defaultMaxListeners = 10;
      function checkListener(listener) {
        if (typeof listener !== "function") {
          throw new TypeError('The "listener" argument must be of type Function. Received type ' + typeof listener);
        }
      }
      Object.defineProperty(EventEmitter5, "defaultMaxListeners", {
        enumerable: true,
        get: function() {
          return defaultMaxListeners;
        },
        set: function(arg) {
          if (typeof arg !== "number" || arg < 0 || NumberIsNaN(arg)) {
            throw new RangeError('The value of "defaultMaxListeners" is out of range. It must be a non-negative number. Received ' + arg + ".");
          }
          defaultMaxListeners = arg;
        }
      });
      EventEmitter5.init = function() {
        if (this._events === void 0 || this._events === Object.getPrototypeOf(this)._events) {
          this._events = /* @__PURE__ */ Object.create(null);
          this._eventsCount = 0;
        }
        this._maxListeners = this._maxListeners || void 0;
      };
      EventEmitter5.prototype.setMaxListeners = function setMaxListeners(n) {
        if (typeof n !== "number" || n < 0 || NumberIsNaN(n)) {
          throw new RangeError('The value of "n" is out of range. It must be a non-negative number. Received ' + n + ".");
        }
        this._maxListeners = n;
        return this;
      };
      function _getMaxListeners(that) {
        if (that._maxListeners === void 0)
          return EventEmitter5.defaultMaxListeners;
        return that._maxListeners;
      }
      EventEmitter5.prototype.getMaxListeners = function getMaxListeners() {
        return _getMaxListeners(this);
      };
      EventEmitter5.prototype.emit = function emit(type) {
        var args = [];
        for (var i = 1; i < arguments.length; i++) args.push(arguments[i]);
        var doError = type === "error";
        var events = this._events;
        if (events !== void 0)
          doError = doError && events.error === void 0;
        else if (!doError)
          return false;
        if (doError) {
          var er;
          if (args.length > 0)
            er = args[0];
          if (er instanceof Error) {
            throw er;
          }
          var err = new Error("Unhandled error." + (er ? " (" + er.message + ")" : ""));
          err.context = er;
          throw err;
        }
        var handler = events[type];
        if (handler === void 0)
          return false;
        if (typeof handler === "function") {
          ReflectApply(handler, this, args);
        } else {
          var len = handler.length;
          var listeners = arrayClone(handler, len);
          for (var i = 0; i < len; ++i)
            ReflectApply(listeners[i], this, args);
        }
        return true;
      };
      function _addListener(target, type, listener, prepend) {
        var m;
        var events;
        var existing;
        checkListener(listener);
        events = target._events;
        if (events === void 0) {
          events = target._events = /* @__PURE__ */ Object.create(null);
          target._eventsCount = 0;
        } else {
          if (events.newListener !== void 0) {
            target.emit(
              "newListener",
              type,
              listener.listener ? listener.listener : listener
            );
            events = target._events;
          }
          existing = events[type];
        }
        if (existing === void 0) {
          existing = events[type] = listener;
          ++target._eventsCount;
        } else {
          if (typeof existing === "function") {
            existing = events[type] = prepend ? [listener, existing] : [existing, listener];
          } else if (prepend) {
            existing.unshift(listener);
          } else {
            existing.push(listener);
          }
          m = _getMaxListeners(target);
          if (m > 0 && existing.length > m && !existing.warned) {
            existing.warned = true;
            var w = new Error("Possible EventEmitter memory leak detected. " + existing.length + " " + String(type) + " listeners added. Use emitter.setMaxListeners() to increase limit");
            w.name = "MaxListenersExceededWarning";
            w.emitter = target;
            w.type = type;
            w.count = existing.length;
            ProcessEmitWarning(w);
          }
        }
        return target;
      }
      EventEmitter5.prototype.addListener = function addListener(type, listener) {
        return _addListener(this, type, listener, false);
      };
      EventEmitter5.prototype.on = EventEmitter5.prototype.addListener;
      EventEmitter5.prototype.prependListener = function prependListener(type, listener) {
        return _addListener(this, type, listener, true);
      };
      function onceWrapper() {
        if (!this.fired) {
          this.target.removeListener(this.type, this.wrapFn);
          this.fired = true;
          if (arguments.length === 0)
            return this.listener.call(this.target);
          return this.listener.apply(this.target, arguments);
        }
      }
      function _onceWrap(target, type, listener) {
        var state = { fired: false, wrapFn: void 0, target, type, listener };
        var wrapped = onceWrapper.bind(state);
        wrapped.listener = listener;
        state.wrapFn = wrapped;
        return wrapped;
      }
      EventEmitter5.prototype.once = function once2(type, listener) {
        checkListener(listener);
        this.on(type, _onceWrap(this, type, listener));
        return this;
      };
      EventEmitter5.prototype.prependOnceListener = function prependOnceListener(type, listener) {
        checkListener(listener);
        this.prependListener(type, _onceWrap(this, type, listener));
        return this;
      };
      EventEmitter5.prototype.removeListener = function removeListener(type, listener) {
        var list, events, position, i, originalListener;
        checkListener(listener);
        events = this._events;
        if (events === void 0)
          return this;
        list = events[type];
        if (list === void 0)
          return this;
        if (list === listener || list.listener === listener) {
          if (--this._eventsCount === 0)
            this._events = /* @__PURE__ */ Object.create(null);
          else {
            delete events[type];
            if (events.removeListener)
              this.emit("removeListener", type, list.listener || listener);
          }
        } else if (typeof list !== "function") {
          position = -1;
          for (i = list.length - 1; i >= 0; i--) {
            if (list[i] === listener || list[i].listener === listener) {
              originalListener = list[i].listener;
              position = i;
              break;
            }
          }
          if (position < 0)
            return this;
          if (position === 0)
            list.shift();
          else {
            spliceOne(list, position);
          }
          if (list.length === 1)
            events[type] = list[0];
          if (events.removeListener !== void 0)
            this.emit("removeListener", type, originalListener || listener);
        }
        return this;
      };
      EventEmitter5.prototype.off = EventEmitter5.prototype.removeListener;
      EventEmitter5.prototype.removeAllListeners = function removeAllListeners(type) {
        var listeners, events, i;
        events = this._events;
        if (events === void 0)
          return this;
        if (events.removeListener === void 0) {
          if (arguments.length === 0) {
            this._events = /* @__PURE__ */ Object.create(null);
            this._eventsCount = 0;
          } else if (events[type] !== void 0) {
            if (--this._eventsCount === 0)
              this._events = /* @__PURE__ */ Object.create(null);
            else
              delete events[type];
          }
          return this;
        }
        if (arguments.length === 0) {
          var keys = Object.keys(events);
          var key;
          for (i = 0; i < keys.length; ++i) {
            key = keys[i];
            if (key === "removeListener") continue;
            this.removeAllListeners(key);
          }
          this.removeAllListeners("removeListener");
          this._events = /* @__PURE__ */ Object.create(null);
          this._eventsCount = 0;
          return this;
        }
        listeners = events[type];
        if (typeof listeners === "function") {
          this.removeListener(type, listeners);
        } else if (listeners !== void 0) {
          for (i = listeners.length - 1; i >= 0; i--) {
            this.removeListener(type, listeners[i]);
          }
        }
        return this;
      };
      function _listeners(target, type, unwrap) {
        var events = target._events;
        if (events === void 0)
          return [];
        var evlistener = events[type];
        if (evlistener === void 0)
          return [];
        if (typeof evlistener === "function")
          return unwrap ? [evlistener.listener || evlistener] : [evlistener];
        return unwrap ? unwrapListeners(evlistener) : arrayClone(evlistener, evlistener.length);
      }
      EventEmitter5.prototype.listeners = function listeners(type) {
        return _listeners(this, type, true);
      };
      EventEmitter5.prototype.rawListeners = function rawListeners(type) {
        return _listeners(this, type, false);
      };
      EventEmitter5.listenerCount = function(emitter, type) {
        if (typeof emitter.listenerCount === "function") {
          return emitter.listenerCount(type);
        } else {
          return listenerCount.call(emitter, type);
        }
      };
      EventEmitter5.prototype.listenerCount = listenerCount;
      function listenerCount(type) {
        var events = this._events;
        if (events !== void 0) {
          var evlistener = events[type];
          if (typeof evlistener === "function") {
            return 1;
          } else if (evlistener !== void 0) {
            return evlistener.length;
          }
        }
        return 0;
      }
      EventEmitter5.prototype.eventNames = function eventNames() {
        return this._eventsCount > 0 ? ReflectOwnKeys(this._events) : [];
      };
      function arrayClone(arr, n) {
        var copy = new Array(n);
        for (var i = 0; i < n; ++i)
          copy[i] = arr[i];
        return copy;
      }
      function spliceOne(list, index) {
        for (; index + 1 < list.length; index++)
          list[index] = list[index + 1];
        list.pop();
      }
      function unwrapListeners(arr) {
        var ret = new Array(arr.length);
        for (var i = 0; i < ret.length; ++i) {
          ret[i] = arr[i].listener || arr[i];
        }
        return ret;
      }
      function once(emitter, name) {
        return new Promise(function(resolve, reject) {
          function errorListener(err) {
            emitter.removeListener(name, resolver);
            reject(err);
          }
          function resolver() {
            if (typeof emitter.removeListener === "function") {
              emitter.removeListener("error", errorListener);
            }
            resolve([].slice.call(arguments));
          }
          ;
          eventTargetAgnosticAddListener(emitter, name, resolver, { once: true });
          if (name !== "error") {
            addErrorHandlerIfEventEmitter(emitter, errorListener, { once: true });
          }
        });
      }
      function addErrorHandlerIfEventEmitter(emitter, handler, flags) {
        if (typeof emitter.on === "function") {
          eventTargetAgnosticAddListener(emitter, "error", handler, flags);
        }
      }
      function eventTargetAgnosticAddListener(emitter, name, listener, flags) {
        if (typeof emitter.on === "function") {
          if (flags.once) {
            emitter.once(name, listener);
          } else {
            emitter.on(name, listener);
          }
        } else if (typeof emitter.addEventListener === "function") {
          emitter.addEventListener(name, function wrapListener(arg) {
            if (flags.once) {
              emitter.removeEventListener(name, wrapListener);
            }
            listener(arg);
          });
        } else {
          throw new TypeError('The "emitter" argument must be of type EventEmitter. Received type ' + typeof emitter);
        }
      }
    }
  });

  // src/index-browser.ts
  var import_buffer = __toESM(require_buffer(), 1);

  // src/consts.ts
  var consts_exports = {};
  __export(consts_exports, {
    ActionType: () => ActionType,
    AlertOperation: () => AlertOperation,
    AlertPayload: () => AlertPayload,
    AlertType: () => AlertType,
    BLECharacteristic: () => BLECharacteristic,
    BLEManufacturerData: () => BLEManufacturerData,
    BLEService: () => BLEService,
    BrakingStyle: () => BrakingStyle,
    ButtonState: () => ButtonState,
    Color: () => Color,
    ColorNames: () => ColorNames,
    CommandFeedback: () => CommandFeedback,
    DeviceType: () => DeviceType,
    DeviceTypeNames: () => DeviceTypeNames,
    DuploTrainBaseSound: () => DuploTrainBaseSound,
    ErrorCode: () => ErrorCode,
    Event: () => Event,
    HWNetWorkCommandType: () => HWNetWorkCommandType,
    HWNetworkFamily: () => HWNetworkFamily,
    HWNetworkSubFamily: () => HWNetworkSubFamily,
    HubPropertyOperation: () => HubPropertyOperation,
    HubPropertyPayload: () => HubPropertyPayload,
    HubPropertyReference: () => HubPropertyReference,
    HubType: () => HubType,
    HubTypeNames: () => HubTypeNames,
    IOTypeID: () => IOTypeID,
    MarioColor: () => MarioColor,
    MarioPantsType: () => MarioPantsType,
    MessageType: () => MessageType,
    ModeInformationType: () => ModeInformationType,
    PortInputFormatSetupSubCommand: () => PortInputFormatSetupSubCommand,
    TiltDirection: () => TiltDirection
  });
  var HubType = /* @__PURE__ */ ((HubType2) => {
    HubType2[HubType2["UNKNOWN"] = 0] = "UNKNOWN";
    HubType2[HubType2["WEDO2_SMART_HUB"] = 1] = "WEDO2_SMART_HUB";
    HubType2[HubType2["MOVE_HUB"] = 2] = "MOVE_HUB";
    HubType2[HubType2["HUB"] = 3] = "HUB";
    HubType2[HubType2["REMOTE_CONTROL"] = 4] = "REMOTE_CONTROL";
    HubType2[HubType2["DUPLO_TRAIN_BASE"] = 5] = "DUPLO_TRAIN_BASE";
    HubType2[HubType2["TECHNIC_MEDIUM_HUB"] = 6] = "TECHNIC_MEDIUM_HUB";
    HubType2[HubType2["MARIO"] = 7] = "MARIO";
    HubType2[HubType2["TECHNIC_SMALL_HUB"] = 8] = "TECHNIC_SMALL_HUB";
    return HubType2;
  })(HubType || {});
  var HubTypeNames = HubType;
  var DeviceType = /* @__PURE__ */ ((DeviceType2) => {
    DeviceType2[DeviceType2["UNKNOWN"] = 0] = "UNKNOWN";
    DeviceType2[DeviceType2["SIMPLE_MEDIUM_LINEAR_MOTOR"] = 1] = "SIMPLE_MEDIUM_LINEAR_MOTOR";
    DeviceType2[DeviceType2["TRAIN_MOTOR"] = 2] = "TRAIN_MOTOR";
    DeviceType2[DeviceType2["LIGHT"] = 8] = "LIGHT";
    DeviceType2[DeviceType2["VOLTAGE_SENSOR"] = 20] = "VOLTAGE_SENSOR";
    DeviceType2[DeviceType2["CURRENT_SENSOR"] = 21] = "CURRENT_SENSOR";
    DeviceType2[DeviceType2["PIEZO_BUZZER"] = 22] = "PIEZO_BUZZER";
    DeviceType2[DeviceType2["HUB_LED"] = 23] = "HUB_LED";
    DeviceType2[DeviceType2["TILT_SENSOR"] = 34] = "TILT_SENSOR";
    DeviceType2[DeviceType2["MOTION_SENSOR"] = 35] = "MOTION_SENSOR";
    DeviceType2[DeviceType2["COLOR_DISTANCE_SENSOR"] = 37] = "COLOR_DISTANCE_SENSOR";
    DeviceType2[DeviceType2["MEDIUM_LINEAR_MOTOR"] = 38] = "MEDIUM_LINEAR_MOTOR";
    DeviceType2[DeviceType2["MOVE_HUB_MEDIUM_LINEAR_MOTOR"] = 39] = "MOVE_HUB_MEDIUM_LINEAR_MOTOR";
    DeviceType2[DeviceType2["MOVE_HUB_TILT_SENSOR"] = 40] = "MOVE_HUB_TILT_SENSOR";
    DeviceType2[DeviceType2["DUPLO_TRAIN_BASE_MOTOR"] = 41] = "DUPLO_TRAIN_BASE_MOTOR";
    DeviceType2[DeviceType2["DUPLO_TRAIN_BASE_SPEAKER"] = 42] = "DUPLO_TRAIN_BASE_SPEAKER";
    DeviceType2[DeviceType2["DUPLO_TRAIN_BASE_COLOR_SENSOR"] = 43] = "DUPLO_TRAIN_BASE_COLOR_SENSOR";
    DeviceType2[DeviceType2["DUPLO_TRAIN_BASE_SPEEDOMETER"] = 44] = "DUPLO_TRAIN_BASE_SPEEDOMETER";
    DeviceType2[DeviceType2["TECHNIC_LARGE_LINEAR_MOTOR"] = 46] = "TECHNIC_LARGE_LINEAR_MOTOR";
    DeviceType2[DeviceType2["TECHNIC_XLARGE_LINEAR_MOTOR"] = 47] = "TECHNIC_XLARGE_LINEAR_MOTOR";
    DeviceType2[DeviceType2["TECHNIC_MEDIUM_ANGULAR_MOTOR"] = 48] = "TECHNIC_MEDIUM_ANGULAR_MOTOR";
    DeviceType2[DeviceType2["TECHNIC_LARGE_ANGULAR_MOTOR"] = 49] = "TECHNIC_LARGE_ANGULAR_MOTOR";
    DeviceType2[DeviceType2["TECHNIC_MEDIUM_HUB_GEST_SENSOR"] = 54] = "TECHNIC_MEDIUM_HUB_GEST_SENSOR";
    DeviceType2[DeviceType2["REMOTE_CONTROL_BUTTON"] = 55] = "REMOTE_CONTROL_BUTTON";
    DeviceType2[DeviceType2["REMOTE_CONTROL_RSSI"] = 56] = "REMOTE_CONTROL_RSSI";
    DeviceType2[DeviceType2["TECHNIC_MEDIUM_HUB_ACCELEROMETER"] = 57] = "TECHNIC_MEDIUM_HUB_ACCELEROMETER";
    DeviceType2[DeviceType2["TECHNIC_MEDIUM_HUB_GYRO_SENSOR"] = 58] = "TECHNIC_MEDIUM_HUB_GYRO_SENSOR";
    DeviceType2[DeviceType2["TECHNIC_MEDIUM_HUB_TILT_SENSOR"] = 59] = "TECHNIC_MEDIUM_HUB_TILT_SENSOR";
    DeviceType2[DeviceType2["TECHNIC_MEDIUM_HUB_TEMPERATURE_SENSOR"] = 60] = "TECHNIC_MEDIUM_HUB_TEMPERATURE_SENSOR";
    DeviceType2[DeviceType2["TECHNIC_COLOR_SENSOR"] = 61] = "TECHNIC_COLOR_SENSOR";
    DeviceType2[DeviceType2["TECHNIC_DISTANCE_SENSOR"] = 62] = "TECHNIC_DISTANCE_SENSOR";
    DeviceType2[DeviceType2["TECHNIC_FORCE_SENSOR"] = 63] = "TECHNIC_FORCE_SENSOR";
    DeviceType2[DeviceType2["TECHNIC_3X3_COLOR_LIGHT_MATRIX"] = 64] = "TECHNIC_3X3_COLOR_LIGHT_MATRIX";
    DeviceType2[DeviceType2["TECHNIC_SMALL_ANGULAR_MOTOR"] = 65] = "TECHNIC_SMALL_ANGULAR_MOTOR";
    DeviceType2[DeviceType2["MARIO_ACCELEROMETER"] = 71] = "MARIO_ACCELEROMETER";
    DeviceType2[DeviceType2["MARIO_BARCODE_SENSOR"] = 73] = "MARIO_BARCODE_SENSOR";
    DeviceType2[DeviceType2["MARIO_PANTS_SENSOR"] = 74] = "MARIO_PANTS_SENSOR";
    DeviceType2[DeviceType2["TECHNIC_MEDIUM_ANGULAR_MOTOR_GREY"] = 75] = "TECHNIC_MEDIUM_ANGULAR_MOTOR_GREY";
    DeviceType2[DeviceType2["TECHNIC_LARGE_ANGULAR_MOTOR_GREY"] = 76] = "TECHNIC_LARGE_ANGULAR_MOTOR_GREY";
    return DeviceType2;
  })(DeviceType || {});
  var DeviceTypeNames = DeviceType;
  var Color = /* @__PURE__ */ ((Color3) => {
    Color3[Color3["BLACK"] = 0] = "BLACK";
    Color3[Color3["PINK"] = 1] = "PINK";
    Color3[Color3["PURPLE"] = 2] = "PURPLE";
    Color3[Color3["BLUE"] = 3] = "BLUE";
    Color3[Color3["LIGHT_BLUE"] = 4] = "LIGHT_BLUE";
    Color3[Color3["CYAN"] = 5] = "CYAN";
    Color3[Color3["GREEN"] = 6] = "GREEN";
    Color3[Color3["YELLOW"] = 7] = "YELLOW";
    Color3[Color3["ORANGE"] = 8] = "ORANGE";
    Color3[Color3["RED"] = 9] = "RED";
    Color3[Color3["WHITE"] = 10] = "WHITE";
    Color3[Color3["NONE"] = 255] = "NONE";
    return Color3;
  })(Color || {});
  var ColorNames = Color;
  var ButtonState = /* @__PURE__ */ ((ButtonState2) => {
    ButtonState2[ButtonState2["PRESSED"] = 2] = "PRESSED";
    ButtonState2[ButtonState2["RELEASED"] = 0] = "RELEASED";
    ButtonState2[ButtonState2["UP"] = 1] = "UP";
    ButtonState2[ButtonState2["DOWN"] = 255] = "DOWN";
    ButtonState2[ButtonState2["STOP"] = 127] = "STOP";
    return ButtonState2;
  })(ButtonState || {});
  var BrakingStyle = /* @__PURE__ */ ((BrakingStyle2) => {
    BrakingStyle2[BrakingStyle2["FLOAT"] = 0] = "FLOAT";
    BrakingStyle2[BrakingStyle2["HOLD"] = 126] = "HOLD";
    BrakingStyle2[BrakingStyle2["BRAKE"] = 127] = "BRAKE";
    return BrakingStyle2;
  })(BrakingStyle || {});
  var DuploTrainBaseSound = /* @__PURE__ */ ((DuploTrainBaseSound2) => {
    DuploTrainBaseSound2[DuploTrainBaseSound2["BRAKE"] = 3] = "BRAKE";
    DuploTrainBaseSound2[DuploTrainBaseSound2["STATION_DEPARTURE"] = 5] = "STATION_DEPARTURE";
    DuploTrainBaseSound2[DuploTrainBaseSound2["WATER_REFILL"] = 7] = "WATER_REFILL";
    DuploTrainBaseSound2[DuploTrainBaseSound2["HORN"] = 9] = "HORN";
    DuploTrainBaseSound2[DuploTrainBaseSound2["STEAM"] = 10] = "STEAM";
    return DuploTrainBaseSound2;
  })(DuploTrainBaseSound || {});
  var BLEManufacturerData = /* @__PURE__ */ ((BLEManufacturerData2) => {
    BLEManufacturerData2[BLEManufacturerData2["DUPLO_TRAIN_BASE_ID"] = 32] = "DUPLO_TRAIN_BASE_ID";
    BLEManufacturerData2[BLEManufacturerData2["MOVE_HUB_ID"] = 64] = "MOVE_HUB_ID";
    BLEManufacturerData2[BLEManufacturerData2["HUB_ID"] = 65] = "HUB_ID";
    BLEManufacturerData2[BLEManufacturerData2["REMOTE_CONTROL_ID"] = 66] = "REMOTE_CONTROL_ID";
    BLEManufacturerData2[BLEManufacturerData2["MARIO_ID"] = 67] = "MARIO_ID";
    BLEManufacturerData2[BLEManufacturerData2["TECHNIC_MEDIUM_HUB_ID"] = 128] = "TECHNIC_MEDIUM_HUB_ID";
    BLEManufacturerData2[BLEManufacturerData2["TECHNIC_SMALL_HUB_ID"] = 131] = "TECHNIC_SMALL_HUB_ID";
    return BLEManufacturerData2;
  })(BLEManufacturerData || {});
  var BLEService = /* @__PURE__ */ ((BLEService2) => {
    BLEService2["WEDO2_SMART_HUB"] = "00001523-1212-efde-1523-785feabcd123";
    BLEService2["WEDO2_SMART_HUB_2"] = "00004f0e-1212-efde-1523-785feabcd123";
    BLEService2["WEDO2_SMART_HUB_3"] = "2a19";
    BLEService2["WEDO2_SMART_HUB_4"] = "180f";
    BLEService2["WEDO2_SMART_HUB_5"] = "180a";
    BLEService2["LPF2_HUB"] = "00001623-1212-efde-1623-785feabcd123";
    return BLEService2;
  })(BLEService || {});
  var BLECharacteristic = /* @__PURE__ */ ((BLECharacteristic2) => {
    BLECharacteristic2["WEDO2_BATTERY"] = "2a19";
    BLECharacteristic2["WEDO2_FIRMWARE_REVISION"] = "2a26";
    BLECharacteristic2["WEDO2_BUTTON"] = "00001526-1212-efde-1523-785feabcd123";
    BLECharacteristic2["WEDO2_PORT_TYPE"] = "00001527-1212-efde-1523-785feabcd123";
    BLECharacteristic2["WEDO2_LOW_VOLTAGE_ALERT"] = "00001528-1212-efde-1523-785feabcd123";
    BLECharacteristic2["WEDO2_HIGH_CURRENT_ALERT"] = "00001529-1212-efde-1523-785feabcd123";
    BLECharacteristic2["WEDO2_LOW_SIGNAL_ALERT"] = "0000152a-1212-efde-1523-785feabcd123";
    BLECharacteristic2["WEDO2_DISCONNECT"] = "0000152b-1212-efde-1523-785feabcd123";
    BLECharacteristic2["WEDO2_SENSOR_VALUE"] = "00001560-1212-efde-1523-785feabcd123";
    BLECharacteristic2["WEDO2_VALUE_FORMAT"] = "00001561-1212-efde-1523-785feabcd123";
    BLECharacteristic2["WEDO2_PORT_TYPE_WRITE"] = "00001563-1212-efde-1523-785feabcd123";
    BLECharacteristic2["WEDO2_MOTOR_VALUE_WRITE"] = "00001565-1212-efde-1523-785feabcd123";
    BLECharacteristic2["WEDO2_NAME_ID"] = "00001524-1212-efde-1523-785feabcd123";
    BLECharacteristic2["LPF2_ALL"] = "00001624-1212-efde-1623-785feabcd123";
    return BLECharacteristic2;
  })(BLECharacteristic || {});
  var MessageType = /* @__PURE__ */ ((MessageType2) => {
    MessageType2[MessageType2["HUB_PROPERTIES"] = 1] = "HUB_PROPERTIES";
    MessageType2[MessageType2["HUB_ACTIONS"] = 2] = "HUB_ACTIONS";
    MessageType2[MessageType2["HUB_ALERTS"] = 3] = "HUB_ALERTS";
    MessageType2[MessageType2["HUB_ATTACHED_IO"] = 4] = "HUB_ATTACHED_IO";
    MessageType2[MessageType2["GENERIC_ERROR_MESSAGES"] = 5] = "GENERIC_ERROR_MESSAGES";
    MessageType2[MessageType2["HW_NETWORK_COMMANDS"] = 8] = "HW_NETWORK_COMMANDS";
    MessageType2[MessageType2["FW_UPDATE_GO_INTO_BOOT_MODE"] = 16] = "FW_UPDATE_GO_INTO_BOOT_MODE";
    MessageType2[MessageType2["FW_UPDATE_LOCK_MEMORY"] = 17] = "FW_UPDATE_LOCK_MEMORY";
    MessageType2[MessageType2["FW_UPDATE_LOCK_STATUS_REQUEST"] = 18] = "FW_UPDATE_LOCK_STATUS_REQUEST";
    MessageType2[MessageType2["FW_LOCK_STATUS"] = 19] = "FW_LOCK_STATUS";
    MessageType2[MessageType2["PORT_INFORMATION_REQUEST"] = 33] = "PORT_INFORMATION_REQUEST";
    MessageType2[MessageType2["PORT_MODE_INFORMATION_REQUEST"] = 34] = "PORT_MODE_INFORMATION_REQUEST";
    MessageType2[MessageType2["PORT_INPUT_FORMAT_SETUP_SINGLE"] = 65] = "PORT_INPUT_FORMAT_SETUP_SINGLE";
    MessageType2[MessageType2["PORT_INPUT_FORMAT_SETUP_COMBINEDMODE"] = 66] = "PORT_INPUT_FORMAT_SETUP_COMBINEDMODE";
    MessageType2[MessageType2["PORT_INFORMATION"] = 67] = "PORT_INFORMATION";
    MessageType2[MessageType2["PORT_MODE_INFORMATION"] = 68] = "PORT_MODE_INFORMATION";
    MessageType2[MessageType2["PORT_VALUE_SINGLE"] = 69] = "PORT_VALUE_SINGLE";
    MessageType2[MessageType2["PORT_VALUE_COMBINEDMODE"] = 70] = "PORT_VALUE_COMBINEDMODE";
    MessageType2[MessageType2["PORT_INPUT_FORMAT_SINGLE"] = 71] = "PORT_INPUT_FORMAT_SINGLE";
    MessageType2[MessageType2["PORT_INPUT_FORMAT_COMBINEDMODE"] = 72] = "PORT_INPUT_FORMAT_COMBINEDMODE";
    MessageType2[MessageType2["VIRTUAL_PORT_SETUP"] = 97] = "VIRTUAL_PORT_SETUP";
    MessageType2[MessageType2["PORT_OUTPUT_COMMAND"] = 129] = "PORT_OUTPUT_COMMAND";
    MessageType2[MessageType2["PORT_OUTPUT_COMMAND_FEEDBACK"] = 130] = "PORT_OUTPUT_COMMAND_FEEDBACK";
    return MessageType2;
  })(MessageType || {});
  var HubPropertyReference = /* @__PURE__ */ ((HubPropertyReference2) => {
    HubPropertyReference2[HubPropertyReference2["ADVERTISING_NAME"] = 1] = "ADVERTISING_NAME";
    HubPropertyReference2[HubPropertyReference2["BUTTON"] = 2] = "BUTTON";
    HubPropertyReference2[HubPropertyReference2["FW_VERSION"] = 3] = "FW_VERSION";
    HubPropertyReference2[HubPropertyReference2["HW_VERSION"] = 4] = "HW_VERSION";
    HubPropertyReference2[HubPropertyReference2["RSSI"] = 5] = "RSSI";
    HubPropertyReference2[HubPropertyReference2["BATTERY_VOLTAGE"] = 6] = "BATTERY_VOLTAGE";
    HubPropertyReference2[HubPropertyReference2["BATTERY_TYPE"] = 7] = "BATTERY_TYPE";
    HubPropertyReference2[HubPropertyReference2["MANUFACTURER_NAME"] = 8] = "MANUFACTURER_NAME";
    HubPropertyReference2[HubPropertyReference2["RADIO_FIRMWARE_VERSION"] = 9] = "RADIO_FIRMWARE_VERSION";
    HubPropertyReference2[HubPropertyReference2["LEGO_WIRELESS_PROTOCOL_VERSION"] = 10] = "LEGO_WIRELESS_PROTOCOL_VERSION";
    HubPropertyReference2[HubPropertyReference2["SYSTEM_TYPE_ID"] = 11] = "SYSTEM_TYPE_ID";
    HubPropertyReference2[HubPropertyReference2["HW_NETWORK_ID"] = 12] = "HW_NETWORK_ID";
    HubPropertyReference2[HubPropertyReference2["PRIMARY_MAC_ADDRESS"] = 13] = "PRIMARY_MAC_ADDRESS";
    HubPropertyReference2[HubPropertyReference2["SECONDARY_MAC_ADDRESS"] = 14] = "SECONDARY_MAC_ADDRESS";
    HubPropertyReference2[HubPropertyReference2["HARDWARE_NETWORK_FAMILY"] = 15] = "HARDWARE_NETWORK_FAMILY";
    return HubPropertyReference2;
  })(HubPropertyReference || {});
  var HubPropertyOperation = /* @__PURE__ */ ((HubPropertyOperation2) => {
    HubPropertyOperation2[HubPropertyOperation2["SET_DOWNSTREAM"] = 1] = "SET_DOWNSTREAM";
    HubPropertyOperation2[HubPropertyOperation2["ENABLE_UPDATES_DOWNSTREAM"] = 2] = "ENABLE_UPDATES_DOWNSTREAM";
    HubPropertyOperation2[HubPropertyOperation2["DISABLE_UPDATES_DOWNSTREAM"] = 3] = "DISABLE_UPDATES_DOWNSTREAM";
    HubPropertyOperation2[HubPropertyOperation2["RESET_DOWNSTREAM"] = 4] = "RESET_DOWNSTREAM";
    HubPropertyOperation2[HubPropertyOperation2["REQUEST_UPDATE_DOWNSTREAM"] = 5] = "REQUEST_UPDATE_DOWNSTREAM";
    HubPropertyOperation2[HubPropertyOperation2["UPDATE_UPSTREAM"] = 6] = "UPDATE_UPSTREAM";
    return HubPropertyOperation2;
  })(HubPropertyOperation || {});
  var HubPropertyPayload = /* @__PURE__ */ ((HubPropertyPayload2) => {
    HubPropertyPayload2[HubPropertyPayload2["ADVERTISING_NAME"] = 1] = "ADVERTISING_NAME";
    HubPropertyPayload2[HubPropertyPayload2["BUTTON_STATE"] = 2] = "BUTTON_STATE";
    HubPropertyPayload2[HubPropertyPayload2["FW_VERSION"] = 3] = "FW_VERSION";
    HubPropertyPayload2[HubPropertyPayload2["HW_VERSION"] = 4] = "HW_VERSION";
    HubPropertyPayload2[HubPropertyPayload2["RSSI"] = 5] = "RSSI";
    HubPropertyPayload2[HubPropertyPayload2["BATTERY_VOLTAGE"] = 6] = "BATTERY_VOLTAGE";
    HubPropertyPayload2[HubPropertyPayload2["BATTERY_TYPE"] = 7] = "BATTERY_TYPE";
    HubPropertyPayload2[HubPropertyPayload2["MANUFACTURER_NAME"] = 8] = "MANUFACTURER_NAME";
    HubPropertyPayload2[HubPropertyPayload2["RADIO_FIRMWARE_VERSION"] = 9] = "RADIO_FIRMWARE_VERSION";
    HubPropertyPayload2[HubPropertyPayload2["LWP_PROTOCOL_VERSION"] = 10] = "LWP_PROTOCOL_VERSION";
    HubPropertyPayload2[HubPropertyPayload2["SYSTEM_TYPE_ID"] = 11] = "SYSTEM_TYPE_ID";
    HubPropertyPayload2[HubPropertyPayload2["HW_NETWORK_ID"] = 12] = "HW_NETWORK_ID";
    HubPropertyPayload2[HubPropertyPayload2["PRIMARY_MAC_ADDRESS"] = 13] = "PRIMARY_MAC_ADDRESS";
    HubPropertyPayload2[HubPropertyPayload2["SECONDARY_MAC_ADDRESS"] = 14] = "SECONDARY_MAC_ADDRESS";
    HubPropertyPayload2[HubPropertyPayload2["HW_NETWORK_FAMILY"] = 15] = "HW_NETWORK_FAMILY";
    return HubPropertyPayload2;
  })(HubPropertyPayload || {});
  var ActionType = /* @__PURE__ */ ((ActionType2) => {
    ActionType2[ActionType2["SWITCH_OFF_HUB"] = 1] = "SWITCH_OFF_HUB";
    ActionType2[ActionType2["DISCONNECT"] = 2] = "DISCONNECT";
    ActionType2[ActionType2["VCC_PORT_CONTROL_ON"] = 3] = "VCC_PORT_CONTROL_ON";
    ActionType2[ActionType2["VCC_PORT_CONTROL_OFF"] = 4] = "VCC_PORT_CONTROL_OFF";
    ActionType2[ActionType2["ACTIVATE_BUSY_INDICATION"] = 5] = "ACTIVATE_BUSY_INDICATION";
    ActionType2[ActionType2["RESET_BUSY_INDICATION"] = 6] = "RESET_BUSY_INDICATION";
    ActionType2[ActionType2["SHUTDOWN"] = 47] = "SHUTDOWN";
    ActionType2[ActionType2["HUB_WILL_SWITCH_OFF"] = 48] = "HUB_WILL_SWITCH_OFF";
    ActionType2[ActionType2["HUB_WILL_DISCONNECT"] = 49] = "HUB_WILL_DISCONNECT";
    ActionType2[ActionType2["HUB_WILL_GO_INTO_BOOT_MODE"] = 50] = "HUB_WILL_GO_INTO_BOOT_MODE";
    return ActionType2;
  })(ActionType || {});
  var AlertType = /* @__PURE__ */ ((AlertType2) => {
    AlertType2[AlertType2["LOW_VOLTAGE"] = 1] = "LOW_VOLTAGE";
    AlertType2[AlertType2["HIGH_CURRENT"] = 2] = "HIGH_CURRENT";
    AlertType2[AlertType2["LOW_SIGNAL_STRENGTH"] = 3] = "LOW_SIGNAL_STRENGTH";
    AlertType2[AlertType2["OVER_POWER_CONDITION"] = 4] = "OVER_POWER_CONDITION";
    return AlertType2;
  })(AlertType || {});
  var AlertOperation = /* @__PURE__ */ ((AlertOperation2) => {
    AlertOperation2[AlertOperation2["LOW_VOLTAGE"] = 1] = "LOW_VOLTAGE";
    AlertOperation2[AlertOperation2["HIGH_CURRENT"] = 2] = "HIGH_CURRENT";
    AlertOperation2[AlertOperation2["LOW_SIGNAL_STRENGTH"] = 3] = "LOW_SIGNAL_STRENGTH";
    AlertOperation2[AlertOperation2["OVER_POWER_CONDITION"] = 4] = "OVER_POWER_CONDITION";
    return AlertOperation2;
  })(AlertOperation || {});
  var AlertPayload = /* @__PURE__ */ ((AlertPayload2) => {
    AlertPayload2[AlertPayload2["STATUS_OK"] = 0] = "STATUS_OK";
    AlertPayload2[AlertPayload2["ALERT"] = 255] = "ALERT";
    return AlertPayload2;
  })(AlertPayload || {});
  var Event = /* @__PURE__ */ ((Event2) => {
    Event2[Event2["DETACHED_IO"] = 0] = "DETACHED_IO";
    Event2[Event2["ATTACHED_IO"] = 1] = "ATTACHED_IO";
    Event2[Event2["ATTACHED_VIRTUAL_IO"] = 2] = "ATTACHED_VIRTUAL_IO";
    return Event2;
  })(Event || {});
  var IOTypeID = /* @__PURE__ */ ((IOTypeID2) => {
    IOTypeID2[IOTypeID2["MOTOR"] = 1] = "MOTOR";
    IOTypeID2[IOTypeID2["SYSTEM_TRAIN_MOTOR"] = 2] = "SYSTEM_TRAIN_MOTOR";
    IOTypeID2[IOTypeID2["BUTTON"] = 5] = "BUTTON";
    IOTypeID2[IOTypeID2["LED_LIGHT"] = 8] = "LED_LIGHT";
    IOTypeID2[IOTypeID2["VOLTAGE"] = 20] = "VOLTAGE";
    IOTypeID2[IOTypeID2["CURRENT"] = 21] = "CURRENT";
    IOTypeID2[IOTypeID2["PIEZO_TONE_SOUND"] = 22] = "PIEZO_TONE_SOUND";
    IOTypeID2[IOTypeID2["RGB_LIGHT"] = 23] = "RGB_LIGHT";
    IOTypeID2[IOTypeID2["EXTERNAL_TILT_SENSOR"] = 34] = "EXTERNAL_TILT_SENSOR";
    IOTypeID2[IOTypeID2["MOTION_SENSOR"] = 35] = "MOTION_SENSOR";
    IOTypeID2[IOTypeID2["VISION_SENSOR"] = 37] = "VISION_SENSOR";
    IOTypeID2[IOTypeID2["EXTERNAL_MOTOR"] = 38] = "EXTERNAL_MOTOR";
    IOTypeID2[IOTypeID2["INTERNAL_MOTOR"] = 39] = "INTERNAL_MOTOR";
    IOTypeID2[IOTypeID2["INTERNAL_TILT"] = 40] = "INTERNAL_TILT";
    return IOTypeID2;
  })(IOTypeID || {});
  var ErrorCode = /* @__PURE__ */ ((ErrorCode2) => {
    ErrorCode2[ErrorCode2["ACK"] = 1] = "ACK";
    ErrorCode2[ErrorCode2["MACK"] = 2] = "MACK";
    ErrorCode2[ErrorCode2["BUFFER_OVERFLOW"] = 3] = "BUFFER_OVERFLOW";
    ErrorCode2[ErrorCode2["TIMEOUT"] = 4] = "TIMEOUT";
    ErrorCode2[ErrorCode2["COMMAND_NOT_RECOGNIZED"] = 5] = "COMMAND_NOT_RECOGNIZED";
    ErrorCode2[ErrorCode2["INVALID_USE"] = 6] = "INVALID_USE";
    ErrorCode2[ErrorCode2["OVERCURRENT"] = 7] = "OVERCURRENT";
    ErrorCode2[ErrorCode2["INTERNAL_ERROR"] = 8] = "INTERNAL_ERROR";
    return ErrorCode2;
  })(ErrorCode || {});
  var HWNetWorkCommandType = /* @__PURE__ */ ((HWNetWorkCommandType2) => {
    HWNetWorkCommandType2[HWNetWorkCommandType2["CONNECTION_REQUEST"] = 2] = "CONNECTION_REQUEST";
    HWNetWorkCommandType2[HWNetWorkCommandType2["FAMILY_REQUEST"] = 3] = "FAMILY_REQUEST";
    HWNetWorkCommandType2[HWNetWorkCommandType2["FAMILY_SET"] = 4] = "FAMILY_SET";
    HWNetWorkCommandType2[HWNetWorkCommandType2["JOIN_DENIED"] = 5] = "JOIN_DENIED";
    HWNetWorkCommandType2[HWNetWorkCommandType2["GET_FAMILY"] = 6] = "GET_FAMILY";
    HWNetWorkCommandType2[HWNetWorkCommandType2["FAMILY"] = 7] = "FAMILY";
    HWNetWorkCommandType2[HWNetWorkCommandType2["GET_SUBFAMILY"] = 8] = "GET_SUBFAMILY";
    HWNetWorkCommandType2[HWNetWorkCommandType2["SUBFAMILY"] = 9] = "SUBFAMILY";
    HWNetWorkCommandType2[HWNetWorkCommandType2["SUBFAMILY_SET"] = 10] = "SUBFAMILY_SET";
    HWNetWorkCommandType2[HWNetWorkCommandType2["GET_EXTENDED_FAMILY"] = 11] = "GET_EXTENDED_FAMILY";
    HWNetWorkCommandType2[HWNetWorkCommandType2["EXTENDED_FAMILY"] = 12] = "EXTENDED_FAMILY";
    HWNetWorkCommandType2[HWNetWorkCommandType2["EXTENDED_FAMILY_SET"] = 13] = "EXTENDED_FAMILY_SET";
    HWNetWorkCommandType2[HWNetWorkCommandType2["RESET_LONG_PRESS_TIMING"] = 14] = "RESET_LONG_PRESS_TIMING";
    return HWNetWorkCommandType2;
  })(HWNetWorkCommandType || {});
  var HWNetworkFamily = /* @__PURE__ */ ((HWNetworkFamily2) => {
    HWNetworkFamily2[HWNetworkFamily2["GREEN"] = 1] = "GREEN";
    HWNetworkFamily2[HWNetworkFamily2["YELLOW"] = 2] = "YELLOW";
    HWNetworkFamily2[HWNetworkFamily2["RED"] = 3] = "RED";
    HWNetworkFamily2[HWNetworkFamily2["BLUE"] = 4] = "BLUE";
    HWNetworkFamily2[HWNetworkFamily2["PURPLE"] = 5] = "PURPLE";
    HWNetworkFamily2[HWNetworkFamily2["LIGHT_BLUE"] = 6] = "LIGHT_BLUE";
    HWNetworkFamily2[HWNetworkFamily2["TEAL"] = 7] = "TEAL";
    HWNetworkFamily2[HWNetworkFamily2["PINK"] = 8] = "PINK";
    HWNetworkFamily2[HWNetworkFamily2["WHITE"] = 0] = "WHITE";
    return HWNetworkFamily2;
  })(HWNetworkFamily || {});
  var HWNetworkSubFamily = /* @__PURE__ */ ((HWNetworkSubFamily2) => {
    HWNetworkSubFamily2[HWNetworkSubFamily2["ONE_FLASH"] = 1] = "ONE_FLASH";
    HWNetworkSubFamily2[HWNetworkSubFamily2["TWO_FLASHES"] = 2] = "TWO_FLASHES";
    HWNetworkSubFamily2[HWNetworkSubFamily2["THREE_FLASHES"] = 3] = "THREE_FLASHES";
    HWNetworkSubFamily2[HWNetworkSubFamily2["FOUR_FLASHES"] = 4] = "FOUR_FLASHES";
    HWNetworkSubFamily2[HWNetworkSubFamily2["FIVE_FLASHES"] = 5] = "FIVE_FLASHES";
    HWNetworkSubFamily2[HWNetworkSubFamily2["SIX_FLASHES"] = 6] = "SIX_FLASHES";
    HWNetworkSubFamily2[HWNetworkSubFamily2["SEVEN_FLASHES"] = 7] = "SEVEN_FLASHES";
    return HWNetworkSubFamily2;
  })(HWNetworkSubFamily || {});
  var ModeInformationType = /* @__PURE__ */ ((ModeInformationType2) => {
    ModeInformationType2[ModeInformationType2["NAME"] = 0] = "NAME";
    ModeInformationType2[ModeInformationType2["RAW"] = 1] = "RAW";
    ModeInformationType2[ModeInformationType2["PCT"] = 2] = "PCT";
    ModeInformationType2[ModeInformationType2["SI"] = 3] = "SI";
    ModeInformationType2[ModeInformationType2["SYMBOL"] = 4] = "SYMBOL";
    ModeInformationType2[ModeInformationType2["MAPPING"] = 5] = "MAPPING";
    ModeInformationType2[ModeInformationType2["USED_INTERNALLY"] = 6] = "USED_INTERNALLY";
    ModeInformationType2[ModeInformationType2["MOTOR_BIAS"] = 7] = "MOTOR_BIAS";
    ModeInformationType2[ModeInformationType2["CAPABILITY_BITS"] = 8] = "CAPABILITY_BITS";
    ModeInformationType2[ModeInformationType2["VALUE_FORMAT"] = 128] = "VALUE_FORMAT";
    return ModeInformationType2;
  })(ModeInformationType || {});
  var PortInputFormatSetupSubCommand = /* @__PURE__ */ ((PortInputFormatSetupSubCommand2) => {
    PortInputFormatSetupSubCommand2[PortInputFormatSetupSubCommand2["SET_MODEANDDATASET_COMBINATIONS"] = 1] = "SET_MODEANDDATASET_COMBINATIONS";
    PortInputFormatSetupSubCommand2[PortInputFormatSetupSubCommand2["LOCK_LPF2_DEVICE_FOR_SETUP"] = 2] = "LOCK_LPF2_DEVICE_FOR_SETUP";
    PortInputFormatSetupSubCommand2[PortInputFormatSetupSubCommand2["UNLOCKANDSTARTWITHMULTIUPDATEENABLED"] = 3] = "UNLOCKANDSTARTWITHMULTIUPDATEENABLED";
    PortInputFormatSetupSubCommand2[PortInputFormatSetupSubCommand2["UNLOCKANDSTARTWITHMULTIUPDATEDISABLED"] = 4] = "UNLOCKANDSTARTWITHMULTIUPDATEDISABLED";
    PortInputFormatSetupSubCommand2[PortInputFormatSetupSubCommand2["NOT_USED"] = 5] = "NOT_USED";
    PortInputFormatSetupSubCommand2[PortInputFormatSetupSubCommand2["RESET_SENSOR"] = 6] = "RESET_SENSOR";
    return PortInputFormatSetupSubCommand2;
  })(PortInputFormatSetupSubCommand || {});
  var MarioPantsType = /* @__PURE__ */ ((MarioPantsType2) => {
    MarioPantsType2[MarioPantsType2["NONE"] = 0] = "NONE";
    MarioPantsType2[MarioPantsType2["PROPELLER"] = 6] = "PROPELLER";
    MarioPantsType2[MarioPantsType2["CAT"] = 17] = "CAT";
    MarioPantsType2[MarioPantsType2["FIRE"] = 18] = "FIRE";
    MarioPantsType2[MarioPantsType2["NORMAL"] = 33] = "NORMAL";
    MarioPantsType2[MarioPantsType2["BUILDER"] = 34] = "BUILDER";
    return MarioPantsType2;
  })(MarioPantsType || {});
  var MarioColor = /* @__PURE__ */ ((MarioColor2) => {
    MarioColor2[MarioColor2["WHITE"] = 4864] = "WHITE";
    MarioColor2[MarioColor2["RED"] = 5376] = "RED";
    MarioColor2[MarioColor2["BLUE"] = 5888] = "BLUE";
    MarioColor2[MarioColor2["YELLOW"] = 6144] = "YELLOW";
    MarioColor2[MarioColor2["BLACK"] = 6656] = "BLACK";
    MarioColor2[MarioColor2["GREEN"] = 9472] = "GREEN";
    MarioColor2[MarioColor2["BROWN"] = 27136] = "BROWN";
    MarioColor2[MarioColor2["CYAN"] = 16897] = "CYAN";
    return MarioColor2;
  })(MarioColor || {});
  var TiltDirection = /* @__PURE__ */ ((TiltDirection2) => {
    TiltDirection2[TiltDirection2["NEUTRAL"] = 0] = "NEUTRAL";
    TiltDirection2[TiltDirection2["BACKWARD"] = 3] = "BACKWARD";
    TiltDirection2[TiltDirection2["RIGHT"] = 5] = "RIGHT";
    TiltDirection2[TiltDirection2["LEFT"] = 7] = "LEFT";
    TiltDirection2[TiltDirection2["FORWARD"] = 9] = "FORWARD";
    TiltDirection2[TiltDirection2["UNKNOWN"] = 10] = "UNKNOWN";
    return TiltDirection2;
  })(TiltDirection || {});
  var CommandFeedback = /* @__PURE__ */ ((CommandFeedback2) => {
    CommandFeedback2[CommandFeedback2["TRANSMISSION_PENDING"] = 0] = "TRANSMISSION_PENDING";
    CommandFeedback2[CommandFeedback2["TRANSMISSION_BUSY"] = 16] = "TRANSMISSION_BUSY";
    CommandFeedback2[CommandFeedback2["TRANSMISSION_DISCARDED"] = 68] = "TRANSMISSION_DISCARDED";
    CommandFeedback2[CommandFeedback2["EXECUTION_PENDING"] = 32] = "EXECUTION_PENDING";
    CommandFeedback2[CommandFeedback2["EXECUTION_BUSY"] = 33] = "EXECUTION_BUSY";
    CommandFeedback2[CommandFeedback2["EXECUTION_DISCARDED"] = 36] = "EXECUTION_DISCARDED";
    CommandFeedback2[CommandFeedback2["EXECUTION_COMPLETED"] = 34] = "EXECUTION_COMPLETED";
    CommandFeedback2[CommandFeedback2["FEEDBACK_MISSING"] = 102] = "FEEDBACK_MISSING";
    CommandFeedback2[CommandFeedback2["FEEDBACK_DISABLED"] = 38] = "FEEDBACK_DISABLED";
    return CommandFeedback2;
  })(CommandFeedback || {});

  // src/webbleabstraction.ts
  var import_debug = __toESM(require_browser(), 1);
  var import_events = __toESM(require_events(), 1);
  var debug = (0, import_debug.default)("bledevice");
  var WebBLEDevice = class extends import_events.EventEmitter {
    constructor(device) {
      super();
      this._name = "";
      this._listeners = {};
      this._characteristics = {};
      this._queue = Promise.resolve();
      this._mailbox = [];
      this._connected = false;
      this._connecting = false;
      this._webBLEServer = device;
      this._uuid = device.device.id;
      this._name = device.device.name;
      device.device.addEventListener("gattserverdisconnected", () => {
        this._connecting = false;
        this._connected = false;
        this.emit("disconnect");
      });
      setTimeout(() => {
        this.emit("discoverComplete");
      }, 2e3);
    }
    get uuid() {
      return this._uuid;
    }
    get name() {
      return this._name;
    }
    get connecting() {
      return this._connecting;
    }
    get connected() {
      return this._connected;
    }
    connect() {
      return new Promise((resolve) => {
        this._connected = true;
        return resolve();
      });
    }
    disconnect() {
      return new Promise((resolve) => {
        this._webBLEServer.device.gatt.disconnect();
        this._connected = false;
        return resolve();
      });
    }
    async discoverCharacteristicsForService(uuid) {
      debug("Service/characteristic discovery started");
      const service = await this._webBLEServer.getPrimaryService(uuid);
      const characteristics = await service.getCharacteristics();
      for (const characteristic of characteristics) {
        this._characteristics[characteristic.uuid] = characteristic;
      }
      debug("Service/characteristic discovery finished");
    }
    subscribeToCharacteristic(uuid, callback) {
      if (this._listeners[uuid]) {
        this._characteristics[uuid].removeEventListener("characteristicvaluechanged", this._listeners[uuid]);
      }
      this._listeners[uuid] = (event) => {
        const buf = Buffer.alloc(event.target.value.buffer.byteLength);
        const view = new Uint8Array(event.target.value.buffer);
        for (let i = 0; i < buf.length; i++) {
          buf[i] = view[i];
        }
        debug("Incoming data", buf);
        return callback(buf);
      };
      this._characteristics[uuid].addEventListener("characteristicvaluechanged", this._listeners[uuid]);
      const mailbox = Array.from(this._mailbox);
      this._mailbox = [];
      for (const data of mailbox) {
        debug("Replayed from mailbox (LPF2_ALL)", data);
        callback(data);
      }
      return this._characteristics[uuid].startNotifications();
    }
    addToCharacteristicMailbox(uuid, data) {
      this._mailbox.push(data);
    }
    readFromCharacteristic(uuid, callback) {
      this._characteristics[uuid].readValue().then((data) => {
        const buf = Buffer.alloc(data.buffer.byteLength);
        const view = new Uint8Array(data.buffer);
        for (let i = 0; i < buf.length; i++) {
          buf[i] = view[i];
        }
        callback(null, buf);
      });
    }
    writeToCharacteristic(uuid, data) {
      return this._queue = this._queue.then(() => this._characteristics[uuid].writeValueWithoutResponse(data));
    }
    _sanitizeUUID(uuid) {
      return uuid.replace(/-/g, "");
    }
  };

  // src/hubs/basehub.ts
  var import_events3 = __toESM(require_events(), 1);

  // src/devices/device.ts
  var import_events2 = __toESM(require_events(), 1);

  // src/portoutputcommand.ts
  var Debug2 = require_browser();
  var debug2 = Debug2("device");
  var PortOutputCommand = class {
    constructor(data, interrupt) {
      this.data = data;
      this.interrupt = interrupt;
      this.state = 0 /* TRANSMISSION_PENDING */;
      this._promise = new Promise((resolve) => {
        this._resolveCallback = () => resolve(this.state);
      });
    }
    get startupAndCompletion() {
      let val = 1;
      if (this.interrupt) val |= 16;
      return val;
    }
    get promise() {
      return this._promise;
    }
    resolve(feedback) {
      debug2("complete command ", this.startupAndCompletion, this.data, " result: ", feedback);
      this.state = feedback;
      this._resolveCallback();
    }
  };

  // src/portoutputsleep.ts
  var PortOutputSleep = class extends PortOutputCommand {
    constructor(duration) {
      super(Buffer.alloc(0), false);
      this.duration = duration;
      this.state = 32 /* EXECUTION_PENDING */;
    }
  };

  // src/devices/device.ts
  var Debug3 = require_browser();
  var debug3 = Debug3("device");
  var Device = class extends import_events2.EventEmitter {
    constructor(hub, portId, modeMap = {}, type = 0 /* UNKNOWN */) {
      super();
      this.autoSubscribe = true;
      this.values = {};
      this._bufferLength = 0;
      this._nextPortOutputCommands = [];
      this._transmittedPortOutputCommands = [];
      this._connected = true;
      this._modeMap = {};
      this._isVirtualPort = false;
      this._hub = hub;
      this._portId = portId;
      this._type = type;
      this._modeMap = modeMap;
      this._isWeDo2SmartHub = this.hub.type === 1 /* WEDO2_SMART_HUB */;
      this._isVirtualPort = this.hub.isPortVirtual(portId);
      const eventAttachListener = (event) => {
        if (event === "detach") {
          return;
        }
        if (this.autoSubscribe) {
          if (this._modeMap[event] !== void 0) {
            this.subscribe(this._modeMap[event]);
          }
        }
      };
      const deviceDetachListener = (device) => {
        if (device.portId === this.portId) {
          this._connected = false;
          this.hub.removeListener("detach", deviceDetachListener);
          this.emit("detach");
        }
      };
      for (const event in this._modeMap) {
        if (this.hub.listenerCount(event) > 0) {
          eventAttachListener(event);
        }
      }
      this.hub.on("newListener", eventAttachListener);
      this.on("newListener", eventAttachListener);
      this.hub.on("detach", deviceDetachListener);
    }
    /**
     * @readonly
     * @property {boolean} connected Check if the device is still attached.
     */
    get connected() {
      return this._connected;
    }
    /**
     * @readonly
     * @property {Hub} hub The Hub the device is attached to.
     */
    get hub() {
      return this._hub;
    }
    get portId() {
      return this._portId;
    }
    /**
     * @readonly
     * @property {string} portName The port the device is attached to.
     */
    get portName() {
      return this.hub.getPortNameForPortId(this.portId);
    }
    /**
     * @readonly
     * @property {number} type The type of the device
     */
    get type() {
      return this._type;
    }
    get typeName() {
      return DeviceTypeNames[this.type];
    }
    /**
     * @readonly
     * @property {number} mode The mode the device is currently in
     */
    get mode() {
      return this._mode;
    }
    get isWeDo2SmartHub() {
      return this._isWeDo2SmartHub;
    }
    /**
     * @readonly
     * @property {boolean} isVirtualPort Is this device attached to a virtual port (ie. a combined device)
     */
    get isVirtualPort() {
      return this._isVirtualPort;
    }
    writeDirect(mode, data, interrupt = false) {
      if (this.isWeDo2SmartHub) {
        return this.send(Buffer.concat([Buffer.from([this.portId, 1, 2]), data]), "00001565-1212-efde-1523-785feabcd123" /* WEDO2_MOTOR_VALUE_WRITE */).then(() => {
          return 38 /* FEEDBACK_DISABLED */;
        });
      } else {
        return this.sendPortOutputCommand(Buffer.concat([Buffer.from([81, mode]), data]), interrupt);
      }
    }
    send(data, characteristic = "00001624-1212-efde-1623-785feabcd123" /* LPF2_ALL */) {
      this._ensureConnected();
      return this.hub.send(data, characteristic);
    }
    subscribe(mode) {
      this._ensureConnected();
      if (mode !== this._mode) {
        this.hub.subscribe(this.portId, this.type, mode);
      }
    }
    unsubscribe(mode) {
      this._ensureConnected();
    }
    receive(message) {
      this.notify("receive", { message });
    }
    notify(event, values) {
      this.values[event] = values;
      this.emit(event, values);
      if (this.hub.listenerCount(event) > 0) {
        this.hub.emit(event, this, values);
      }
    }
    requestUpdate() {
      this.send(Buffer.from([33, this.portId, 0]));
    }
    setMode(mode) {
      this._mode = mode;
    }
    transmitNextPortOutputCommand() {
      if (!this.connected) {
        this._transmittedPortOutputCommands.forEach((command) => command.resolve(102 /* FEEDBACK_MISSING */));
        this._transmittedPortOutputCommands = [];
        this._nextPortOutputCommands.forEach((command) => command.resolve(68 /* TRANSMISSION_DISCARDED */));
        this._nextPortOutputCommands = [];
        return;
      }
      if (!this._nextPortOutputCommands.length) return;
      const nextCommand = this._nextPortOutputCommands[0];
      if (nextCommand instanceof PortOutputSleep) {
        if (nextCommand.state === 32 /* EXECUTION_PENDING */) {
          nextCommand.state = 33 /* EXECUTION_BUSY */;
          debug3("sleep command ", nextCommand.duration);
          setTimeout(() => {
            if (nextCommand.state !== 33 /* EXECUTION_BUSY */) return;
            const command = this._nextPortOutputCommands.shift();
            if (command) command.resolve(34 /* EXECUTION_COMPLETED */);
            this.transmitNextPortOutputCommand();
          }, nextCommand.duration);
        }
        return;
      }
      if (this._bufferLength !== this._transmittedPortOutputCommands.length) return;
      if (this._bufferLength < 2 || nextCommand.interrupt) {
        if (nextCommand.state === 0 /* TRANSMISSION_PENDING */) {
          nextCommand.state = 16 /* TRANSMISSION_BUSY */;
          debug3("transmit command ", nextCommand.startupAndCompletion, nextCommand.data);
          this.send(Buffer.concat([Buffer.from([129, this.portId, nextCommand.startupAndCompletion]), nextCommand.data])).then(() => {
            if (nextCommand.state !== 16 /* TRANSMISSION_BUSY */) return;
            const command = this._nextPortOutputCommands.shift();
            if (command instanceof PortOutputCommand) this._transmittedPortOutputCommands.push(command);
          });
          this.transmitNextPortOutputCommand();
        }
      }
    }
    sendPortOutputCommand(data, interrupt = false) {
      if (this.isWeDo2SmartHub) {
        throw new Error("PortOutputCommands are not available on the WeDo 2.0 Smart Hub");
        return;
      }
      const command = new PortOutputCommand(data, interrupt);
      if (interrupt) {
        this._nextPortOutputCommands.forEach((command2) => {
          if (command2.state !== 16 /* TRANSMISSION_BUSY */) {
            command2.resolve(68 /* TRANSMISSION_DISCARDED */);
          }
        });
        this._nextPortOutputCommands = this._nextPortOutputCommands.filter((command2) => command2.state === 16 /* TRANSMISSION_BUSY */);
      }
      this._nextPortOutputCommands.push(command);
      process.nextTick(() => this.transmitNextPortOutputCommand());
      return command.promise;
    }
    addPortOutputSleep(duration) {
      const command = new PortOutputSleep(duration);
      this._nextPortOutputCommands.push(command);
      process.nextTick(() => this.transmitNextPortOutputCommand());
      return command.promise;
    }
    finish(message) {
      debug3("recieved command feedback ", message);
      if ((message & 8) === 8) this._bufferLength = 0;
      else if ((message & 1) === 1) this._bufferLength = 1;
      else if ((message & 16) === 16) this._bufferLength = 2;
      const completed = (message & 2) === 2;
      const discarded = (message & 4) === 4;
      switch (this._transmittedPortOutputCommands.length) {
        case 0:
          break;
        case 1:
          if (!this._bufferLength && completed && !discarded) {
            this._complete();
          } else if (!this._bufferLength && !completed && discarded) {
            this._discard();
          } else if (this._bufferLength && !completed && !discarded) {
            this._busy();
          } else {
            this._missing();
          }
          break;
        case 2:
          if (!this._bufferLength && completed && discarded) {
            this._discard();
            this._complete();
          } else if (!this._bufferLength && completed && !discarded) {
            this._complete();
            this._complete();
          } else if (!this._bufferLength && !completed && discarded) {
            this._discard();
            this._discard();
          } else if (this._bufferLength === 1 && completed && !discarded) {
            this._complete();
            this._busy();
          } else if (this._bufferLength === 1 && !completed && discarded) {
            this._discard();
            this._busy();
          } else if (this._bufferLength === 1 && completed && discarded) {
            this._missing();
            this._busy();
          } else if (this._bufferLength === 2 && !completed && !discarded) {
            this._busy();
            this._pending();
          } else {
            this._missing();
            this._missing();
          }
          break;
        case 3:
          if (!this._bufferLength && completed && discarded) {
            this._discard();
            this._discard();
            this._complete();
          } else if (!this._bufferLength && completed && !discarded) {
            this._complete();
            this._complete();
            this._complete();
          } else if (!this._bufferLength && !completed && discarded) {
            this._discard();
            this._discard();
            this._discard();
          } else if (this._bufferLength === 1 && completed && discarded) {
            this._discard();
            this._complete();
            this._busy();
          } else if (this._bufferLength === 1 && completed && !discarded) {
            this._complete();
            this._complete();
            this._busy();
          } else if (this._bufferLength === 1 && !completed && discarded) {
            this._discard();
            this._discard();
            this._busy();
          } else if (this._bufferLength === 1 && !completed && !discarded) {
            this._missing();
            this._missing();
            this._busy();
          } else {
            this._missing();
            this._missing();
            this._missing();
          }
          break;
      }
      this.transmitNextPortOutputCommand();
    }
    _ensureConnected() {
      if (!this.connected) {
        throw new Error("Device is not connected");
      }
    }
    _complete() {
      const command = this._transmittedPortOutputCommands.shift();
      if (command) command.resolve(34 /* EXECUTION_COMPLETED */);
    }
    _discard() {
      const command = this._transmittedPortOutputCommands.shift();
      if (command) command.resolve(36 /* EXECUTION_DISCARDED */);
    }
    _missing() {
      const command = this._transmittedPortOutputCommands.shift();
      if (command) command.resolve(102 /* FEEDBACK_MISSING */);
    }
    _busy() {
      const command = this._transmittedPortOutputCommands[0];
      if (command) command.state = 33 /* EXECUTION_BUSY */;
    }
    _pending() {
      const command = this._transmittedPortOutputCommands[1];
      if (command) command.state = 32 /* EXECUTION_PENDING */;
    }
  };

  // src/utils.ts
  var isWebBluetooth = !!(typeof navigator !== "undefined" && navigator && navigator.bluetooth);
  var toHex = (value, length = 2) => {
    return value.toString(16).padStart(length, "0");
  };
  var toBin = (value, length = 8) => {
    return value.toString(2).padStart(length, "0");
  };
  var mapSpeed = (speed) => {
    if (speed === 127) {
      return 127;
    }
    if (speed > 100) {
      speed = 100;
    } else if (speed < -100) {
      speed = -100;
    }
    return speed;
  };
  var decodeVersion = (version) => {
    const parts = version.toString(16).padStart(8, "0");
    return [parts[0], parts[1], parts.substring(2, 4), parts.substring(4)].join(".");
  };
  var decodeMACAddress = (address) => {
    return Array.from(address).map((part) => toHex(part, 2)).join(":");
  };
  var normalizeAngle = (angle) => {
    if (angle >= 180) {
      return angle - 360 * ((angle + 180) / 360);
    } else if (angle < -180) {
      return angle + 360 * ((180 - angle) / 360);
    }
    return angle;
  };
  var calculateRamp = (fromPower, toPower, time) => {
    const steps = Math.abs(toPower - fromPower);
    if (steps === 0 || time <= 0) {
      return [toPower];
    }
    let delay = time / steps;
    let increment = 1;
    if (delay < 50) {
      increment = 50 / delay;
      delay = 50;
    }
    if (fromPower > toPower) {
      increment = -increment;
    }
    return Array(Math.round(time / delay)).fill(0).map((element, index) => {
      let value = Math.round(fromPower + (index + 1) * increment);
      if (toPower > fromPower && value > toPower) {
        value = toPower;
      } else if (fromPower > toPower && value < toPower) {
        value = toPower;
      }
      return value;
    });
  };
  var parseColor = (color) => {
    if (color === 1 || color === 5) {
      color = color + 1;
    }
    return color;
  };

  // src/devices/colordistancesensor.ts
  var ColorDistanceSensor = class extends Device {
    constructor(hub, portId) {
      super(hub, portId, ModeMap, 37 /* COLOR_DISTANCE_SENSOR */);
    }
    receive(message) {
      const mode = this._mode;
      switch (mode) {
        case 0 /* COLOR */:
          if (message[this.isWeDo2SmartHub ? 2 : 4] <= 10) {
            const color = parseColor(message[this.isWeDo2SmartHub ? 2 : 4]);
            this.notify("color", { color });
          }
          break;
        case 1 /* DISTANCE */:
          if (this.isWeDo2SmartHub) {
            break;
          }
          if (message[4] <= 10) {
            let distance2 = Math.floor(message[4] * 25.4) - 20;
            if (distance2 < 0) {
              distance2 = 0;
            }
            this.notify("distance", { distance: distance2 });
          }
          break;
        case 2 /* DISTANCE_COUNT */:
          if (this.isWeDo2SmartHub) {
            break;
          }
          if (message.length !== 8) {
            break;
          }
          const count = message.readUInt32LE(4);
          this.notify("distanceCount", { count });
          break;
        case 3 /* REFLECT */:
          if (this.isWeDo2SmartHub) {
            break;
          }
          const reflect = message[4];
          this.notify("reflect", { reflect });
          break;
        case 4 /* AMBIENT */:
          if (this.isWeDo2SmartHub) {
            break;
          }
          const ambient = message[4];
          this.notify("ambient", { ambient });
          break;
        case 6 /* RGB_I */:
          if (this.isWeDo2SmartHub) {
            break;
          }
          if (message.length !== 10) {
            break;
          }
          const red = message.readUInt16LE(4);
          const green = message.readUInt16LE(6);
          const blue = message.readUInt16LE(8);
          this.notify("rgbIntensity", { red, green, blue });
          break;
        case 8 /* COLOR_AND_DISTANCE */:
          if (this.isWeDo2SmartHub) {
            break;
          }
          let distance = message[5];
          const partial = message[7];
          if (partial > 0) {
            distance += 1 / partial;
          }
          distance = Math.floor(distance * 25.4) - 20;
          if (message[4] <= 10) {
            const color = message[4];
            this.notify("colorAndDistance", { color, distance });
          }
          break;
      }
    }
    /**
     * Switches the IR receiver into extended channel mode. After setting this, use channels 5-8 instead of 1-4 for this receiver.
     *
     * NOTE: Calling this with channel 5-8 with switch off extended channel mode for this receiver.
     * @param {number} channel Channel number, between 1-8
     * @returns {Promise<CommandFeedback>} Resolved upon completion of the command.
     */
    setPFExtendedChannel(channel) {
      let address = 0;
      if (channel >= 4) {
        channel -= 4;
        address = 1;
      }
      const message = Buffer.alloc(2);
      message[0] = (channel - 1 << 4) + (address << 3);
      message[1] = 6 << 4;
      return this.sendPFIRMessage(message);
    }
    /**
     * Set the power of a Power Functions motor via IR
     * @param {number} channel Channel number, between 1-4
     * @param {string} output Outport port, "RED" (A) or "BLUE" (B)
     * @param {number} power -7 (full reverse) to 7 (full forward). 0 is stop. 8 is brake.
     * @returns {Promise<CommandFeedback>} Resolved upon completion of the command.
     */
    setPFPower(channel, output, power) {
      let address = 0;
      if (channel > 4) {
        channel -= 4;
        address = 1;
      }
      const message = Buffer.alloc(2);
      message[0] = (channel - 1 << 4) + (address << 3) + (output === "RED" ? 4 : 5);
      message[1] = this._pfPowerToPWM(power) << 4;
      return this.sendPFIRMessage(message);
    }
    /**
     * Start Power Functions motors running via IR
     *
     * NOTE: This command is designed for bang-bang style operation. To keep the motors running, the sensor needs to be within range of the IR receiver constantly.
     * @param {number} channel Channel number, between 1-4
     * @param {number} powerBlue -7 (full reverse) to 7 (full forward). 0 is stop. 8 is brake.
     * @param {number} powerRed -7 (full reverse) to 7 (full forward). 0 is stop. 8 is brake.
     * @returns {Promise<CommandFeedback>} Resolved upon completion of the command.
     */
    startPFMotors(channel, powerBlue, powerRed) {
      let address = 0;
      if (channel > 4) {
        channel -= 4;
        address = 1;
      }
      const message = Buffer.alloc(2);
      message[0] = (channel - 1 + 4 + (address << 3) << 4) + this._pfPowerToPWM(powerBlue);
      message[1] = this._pfPowerToPWM(powerRed) << 4;
      return this.sendPFIRMessage(message);
    }
    /**
     * Send a raw Power Functions IR command
     * @param {Buffer} message 2 byte payload making up a Power Functions protocol command. NOTE: Only specify nibbles 1-3, nibble 4 should be zeroed.
     * @returns {Promise<CommandFeedback>} Resolved upon completion of the command.
     */
    sendPFIRMessage(message) {
      if (this.isWeDo2SmartHub) {
        throw new Error("Power Functions IR is not available on the WeDo 2.0 Smart Hub");
      } else {
        const payload = Buffer.alloc(2);
        payload[0] = (message[0] << 4) + (message[1] >> 4);
        payload[1] = message[0] >> 4;
        this.subscribe(7 /* PF_IR */);
        return this.writeDirect(7, payload);
      }
    }
    /**
     * Set the color of the LED on the sensor via a color value.
     * @param {Color} color
     * @returns {Promise<CommandFeedback>} Resolved upon completion of the command.
     */
    setColor(color) {
      if (color === false) {
        color = 0;
      }
      if (this.isWeDo2SmartHub) {
        throw new Error("Setting LED color is not available on the WeDo 2.0 Smart Hub");
      } else {
        this.subscribe(5 /* LED */);
        return this.writeDirect(5, Buffer.from([color]));
      }
    }
    /**
     * Set the distance count value.
     * @param {number} count distance count between 0 and 2^32
     * @returns {Promise<CommandFeedback>} Resolved upon completion of the command.
     */
    setDistanceCount(count) {
      if (this.isWeDo2SmartHub) {
        throw new Error("Setting distance count is not available on the WeDo 2.0 Smart Hub");
      } else {
        const payload = Buffer.alloc(4);
        payload.writeUInt32LE(count % 2 ** 32);
        return this.writeDirect(2, payload);
      }
    }
    _pfPowerToPWM(power) {
      return power & 15;
    }
  };
  var ModeMap = {
    "color": 0 /* COLOR */,
    "distance": 1 /* DISTANCE */,
    "distanceCount": 2 /* DISTANCE_COUNT */,
    "reflect": 3 /* REFLECT */,
    "ambient": 4 /* AMBIENT */,
    "rgbIntensity": 6 /* RGB_I */,
    "colorAndDistance": 8 /* COLOR_AND_DISTANCE */
  };

  // src/devices/currentsensor.ts
  var CurrentSensor = class extends Device {
    constructor(hub, portId) {
      super(hub, portId, ModeMap2, 21 /* CURRENT_SENSOR */);
    }
    receive(message) {
      const mode = this.mode;
      switch (mode) {
        case 0 /* CURRENT */:
          if (this.isWeDo2SmartHub) {
            const current = message.readInt16LE(2) / 1e3;
            this.notify("current", { current });
          } else {
            let maxCurrentValue = MaxCurrentValue[this.hub.type];
            if (maxCurrentValue === void 0) {
              maxCurrentValue = MaxCurrentValue[0 /* UNKNOWN */];
            }
            let maxCurrentRaw = MaxCurrentRaw[this.hub.type];
            if (maxCurrentRaw === void 0) {
              maxCurrentRaw = MaxCurrentRaw[0 /* UNKNOWN */];
            }
            const current = message.readUInt16LE(4) * maxCurrentValue / maxCurrentRaw;
            this.notify("current", { current });
          }
          break;
      }
    }
  };
  var ModeMap2 = {
    "current": 0 /* CURRENT */
  };
  var MaxCurrentValue = {
    [0 /* UNKNOWN */]: 2444,
    [6 /* TECHNIC_MEDIUM_HUB */]: 4175
  };
  var MaxCurrentRaw = {
    [0 /* UNKNOWN */]: 4095
  };

  // src/devices/duplotrainbasecolorsensor.ts
  var DuploTrainBaseColorSensor = class extends Device {
    constructor(hub, portId) {
      super(hub, portId, ModeMap3, 43 /* DUPLO_TRAIN_BASE_COLOR_SENSOR */);
    }
    receive(message) {
      const mode = this._mode;
      switch (mode) {
        case 0 /* INTENSITY */:
          const intensity = message[4];
          this.notify("intensity", { intensity });
          break;
        case 1 /* COLOR */:
          if (message[4] <= 10) {
            const color = parseColor(message[4]);
            this.notify("color", { color });
          }
          break;
        case 2 /* REFLECTIVITY */:
          const reflect = message[4];
          this.notify("reflect", { reflect });
          break;
        case 3 /* RGB */:
          const red = Math.floor(message.readUInt16LE(4) / 4);
          const green = Math.floor(message.readUInt16LE(6) / 4);
          const blue = Math.floor(message.readUInt16LE(8) / 4);
          this.notify("rgb", { red, green, blue });
          break;
      }
    }
  };
  var ModeMap3 = {
    "intensity": 0 /* INTENSITY */,
    "color": 1 /* COLOR */,
    "reflect": 2 /* REFLECTIVITY */,
    "rgb": 3 /* RGB */
  };

  // src/devices/basicmotor.ts
  var BasicMotor = class extends Device {
    constructor(hub, portId, modeMap, type = 0 /* UNKNOWN */) {
      super(hub, portId, modeMap, type);
    }
    /**
     * Set the motor power.
     * @param {number} power For forward, a value between 1 - 100 should be set. For reverse, a value between -1 to -100. Stop is 0.
     * @param {boolean} [interrupt=false] If true, previous commands are discarded.
     * @returns {Promise<CommandFeedback>} Resolved upon completion of command.
     */
    setPower(power, interrupt = false) {
      return this.writeDirect(0, Buffer.from([mapSpeed(power)]), interrupt);
    }
    /**
     * Ramp the motor power.
     * @param {number} fromPower For forward, a value between 1 - 100 should be set. For reverse, a value between -1 to -100. Stop is 0.
     * @param {number} toPower For forward, a value between 1 - 100 should be set. For reverse, a value between -1 to -100. Stop is 0.
     * @param {number} time How long the ramp should last (in milliseconds).
     * @returns {Promise<CommandFeedback>} Resolved upon completion of command.
     */
    rampPower(fromPower, toPower, time) {
      const powerValues = calculateRamp(fromPower, toPower, time);
      powerValues.forEach((value) => {
        this.setPower(value);
        this.addPortOutputSleep(Math.round(time / powerValues.length));
      });
      return this.setPower(toPower);
    }
    /**
     * Stop the motor. Previous commands that have not completed are discarded.
     * @returns {Promise<CommandFeedback>} Resolved upon completion of command.
     */
    stop() {
      return this.setPower(0, true);
    }
    /**
     * Brake the motor. Previous commands that have not completed are discarded.
     * @returns {Promise<CommandFeedback>} Resolved upon completion of command.
     */
    brake() {
      return this.setPower(127 /* BRAKE */, true);
    }
  };

  // src/devices/duplotrainbasemotor.ts
  var DuploTrainBaseMotor = class extends BasicMotor {
    constructor(hub, portId) {
      super(hub, portId, {}, 41 /* DUPLO_TRAIN_BASE_MOTOR */);
    }
  };

  // src/devices/duplotrainbasespeaker.ts
  var DuploTrainBaseSpeaker = class extends Device {
    constructor(hub, portId) {
      super(hub, portId, {}, 42 /* DUPLO_TRAIN_BASE_SPEAKER */);
    }
    /**
     * Play a built-in train sound.
     * @param {DuploTrainBaseSound} sound
     * @returns {Promise<CommandFeedback>} Resolved upon completion of command.
     */
    playSound(sound) {
      this.subscribe(1 /* SOUND */);
      return this.writeDirect(1, Buffer.from([sound]));
    }
    /**
     * Play a built-in system tone.
     * @param {number} tone
     * @returns {Promise<CommandFeedback>} Resolved upon completion of command.
     */
    playTone(tone) {
      this.subscribe(2 /* TONE */);
      return this.writeDirect(2, Buffer.from([tone]));
    }
  };

  // src/devices/duplotrainbasespeedometer.ts
  var DuploTrainBaseSpeedometer = class extends Device {
    constructor(hub, portId) {
      super(hub, portId, ModeMap4, 44 /* DUPLO_TRAIN_BASE_SPEEDOMETER */);
    }
    receive(message) {
      const mode = this._mode;
      switch (mode) {
        case 0 /* SPEED */:
          const speed = message.readInt16LE(4);
          this.notify("speed", { speed });
          break;
      }
    }
  };
  var ModeMap4 = {
    "speed": 0 /* SPEED */
  };

  // src/devices/hubled.ts
  var HubLED = class extends Device {
    constructor(hub, portId) {
      super(hub, portId, {}, 23 /* HUB_LED */);
    }
    /**
     * Set the color of the LED on the Hub via a color value.
     * @param {Color} color
     * @returns {Promise<CommandFeedback>} Resolved upon completion of command.
     */
    setColor(color) {
      if (typeof color === "boolean") {
        color = 0;
      }
      if (this.isWeDo2SmartHub) {
        return new Promise((resolve) => {
          this.send(Buffer.from([6, 23, 1, 1]), "00001563-1212-efde-1523-785feabcd123" /* WEDO2_PORT_TYPE_WRITE */);
          this.send(Buffer.from([6, 4, 1, Number(color)]), "00001565-1212-efde-1523-785feabcd123" /* WEDO2_MOTOR_VALUE_WRITE */);
          return resolve(38 /* FEEDBACK_DISABLED */);
        });
      } else {
        this.subscribe(0 /* COLOR */);
        return this.writeDirect(0, Buffer.from([color]));
      }
    }
    /**
     * Set the color of the LED on the Hub via RGB values.
     * @param {number} red
     * @param {number} green
     * @param {number} blue
     * @returns {Promise<CommandFeedback>} Resolved upon completion of command.
     */
    setRGB(red, green, blue) {
      if (this.isWeDo2SmartHub) {
        return new Promise((resolve) => {
          this.send(Buffer.from([6, 23, 1, 2]), "00001563-1212-efde-1523-785feabcd123" /* WEDO2_PORT_TYPE_WRITE */);
          this.send(Buffer.from([6, 4, 3, red, green, blue]), "00001565-1212-efde-1523-785feabcd123" /* WEDO2_MOTOR_VALUE_WRITE */);
          resolve(38 /* FEEDBACK_DISABLED */);
        });
      } else {
        this.subscribe(1 /* RGB */);
        return this.writeDirect(1, Buffer.from([red, green, blue]));
      }
    }
  };

  // src/devices/light.ts
  var Light = class extends Device {
    constructor(hub, portId) {
      super(hub, portId, {}, 8 /* LIGHT */);
    }
    /**
     * Set the light brightness.
     * @param {number} brightness Brightness value between 0-100 (0 is off)
     * @param {number} brightness Brightness value between 0-100 (0 is off)
     * @param {boolean} [interrupt=false] If true, previous commands are discarded.
     * @returns {Promise<CommandFeedback>} Resolved upon completion of command.
     */
    setBrightness(brightness, interrupt = false) {
      return this.writeDirect(0, Buffer.from([brightness]), interrupt);
    }
    /**
     * Ramp the light brightness.
     * @param {number} fromBrightness Brightness value between 0-100 (0 is off)
     * @param {number} toBrightness Brightness value between 0-100 (0 is off)
     * @param {number} time How long the ramp should last (in milliseconds).
     * @returns {Promise<CommandFeedback>} Resolved upon completion of command.
     */
    rampBrightness(fromBrightness, toBrightness, time) {
      const powerValues = calculateRamp(fromBrightness, toBrightness, time);
      powerValues.forEach((value) => {
        this.setBrightness(value);
        this.addPortOutputSleep(Math.round(time / powerValues.length));
      });
      return this.setBrightness(toBrightness);
    }
  };

  // src/devices/marioaccelerometer.ts
  var MarioAccelerometer = class extends Device {
    constructor(hub, portId) {
      super(hub, portId, ModeMap5, 71 /* MARIO_ACCELEROMETER */);
    }
    receive(message) {
      const mode = this._mode;
      switch (mode) {
        case 0 /* ACCEL */:
          const x = message[4];
          const y = message[5];
          const z = message[6];
          this.notify("accel", { x, y, z });
          break;
        case 1 /* GEST */:
          const gesture = message[4];
          this.notify("gesture", { gesture });
          break;
      }
    }
  };
  var ModeMap5 = {
    "accel": 0 /* ACCEL */,
    "gesture": 1 /* GEST */
  };

  // src/devices/mariobarcodesensor.ts
  var MarioBarcodeSensor = class extends Device {
    constructor(hub, portId) {
      super(hub, portId, ModeMap6, 73 /* MARIO_BARCODE_SENSOR */);
    }
    receive(message) {
      const mode = this._mode;
      switch (mode) {
        case 0 /* BARCODE */:
          const barcode = message.readUInt16LE(4);
          const color = message.readUInt16LE(6);
          if (color === 65535) {
            this.notify("barcode", { barcode });
          } else if (barcode === 65535) {
            this.notify("barcode", { color });
          }
          break;
        case 1 /* RGB */:
          const r = message[4];
          const g = message[5];
          const b = message[6];
          this.notify("rgb", { r, g, b });
          break;
      }
    }
  };
  var ModeMap6 = {
    "barcode": 0 /* BARCODE */,
    "rgb": 1 /* RGB */
  };

  // src/devices/mariopantssensor.ts
  var MarioPantsSensor = class extends Device {
    constructor(hub, portId) {
      super(hub, portId, ModeMap7, 74 /* MARIO_PANTS_SENSOR */);
    }
    receive(message) {
      const mode = this._mode;
      switch (mode) {
        case 0 /* PANTS */:
          const pants = message[4];
          this.notify("pants", { pants });
          break;
      }
    }
  };
  var ModeMap7 = {
    "pants": 0 /* PANTS */
  };

  // src/devices/tachomotor.ts
  var TachoMotor = class extends BasicMotor {
    constructor(hub, portId, modeMap = {}, type = 0 /* UNKNOWN */) {
      super(hub, portId, Object.assign({}, modeMap, ModeMap8), type);
      this._brakeStyle = 127 /* BRAKE */;
      this._maxPower = 100;
      this.useAccelerationProfile = true;
      this.useDecelerationProfile = true;
    }
    receive(message) {
      const mode = this._mode;
      switch (mode) {
        case 2 /* ROTATION */:
          const degrees = message.readInt32LE(this.isWeDo2SmartHub ? 2 : 4);
          this.notify("rotate", { degrees });
          break;
      }
    }
    /**
     * Set the braking style of the motor.
     *
     * Note: This applies to setSpeed, rotateByDegrees, and gotoAngle.
     * @param {number} style Either BRAKE or HOLD
     */
    setBrakingStyle(style) {
      this._brakeStyle = style;
    }
    /**
     * Set the max power of the motor.
     *
     * Note: This applies to setSpeed, rotateByDegrees, and gotoAngle.
     * @param {number} maxPower Maximum power level (0-100)
     */
    setMaxPower(maxPower) {
      this._maxPower = maxPower;
    }
    /**
     * Set the global acceleration time
     * @param {number} time How long acceleration should last (in milliseconds).
     * @param {number} profile 0 by default
     * @param {boolean} [interrupt=false] If true, previous commands are discarded.
     * @returns {Promise<CommandFeedback>} Resolved upon completion of command (i.e. once the motor is finished).
     */
    setAccelerationTime(time, profile = 0, interrupt = false) {
      const message = Buffer.from([5, 0, 0, profile]);
      message.writeUInt16LE(time, 1);
      return this.sendPortOutputCommand(message, interrupt);
    }
    /**
     * Set the global deceleration time
     * @param {number} time How long deceleration should last (in milliseconds).
     * @param {number} profile 0 by default
     * @param {boolean} [interrupt=false] If true, previous commands are discarded.
     * @returns {Promise<CommandFeedback>} Resolved upon completion of command (i.e. once the motor is finished).
     */
    setDecelerationTime(time, profile = 0, interrupt = true) {
      const message = Buffer.from([6, 0, 0, profile]);
      message.writeUInt16LE(time, 1);
      return this.sendPortOutputCommand(message, interrupt);
    }
    /**
     * Set the motor speed.
     * @param {number} speed For forward, a value between 1 - 100 should be set. For reverse, a value between -1 to -100. Stop is 0.
     * @param {number} time How long the motor should run for (in milliseconds).
     * @param {boolean} [interrupt=false] If true, previous commands are discarded.
     * @returns {Promise<CommandFeedback>} Resolved upon completion of command (i.e. once the motor is finished).
     */
    setSpeed(speed, time, interrupt = false) {
      if (!this.isVirtualPort && speed instanceof Array) {
        throw new Error("Only virtual ports can accept multiple speeds");
      }
      if (this.isWeDo2SmartHub) {
        throw new Error("Motor speed is not available on the WeDo 2.0 Smart Hub");
      }
      if (speed === void 0 || speed === null) {
        speed = 100;
      }
      let message;
      if (time !== void 0) {
        if (speed instanceof Array) {
          message = Buffer.from([10, 0, 0, mapSpeed(speed[0]), mapSpeed(speed[1]), this._maxPower, this._brakeStyle, this.useProfile()]);
        } else {
          message = Buffer.from([9, 0, 0, mapSpeed(speed), this._maxPower, this._brakeStyle, this.useProfile()]);
        }
        message.writeUInt16LE(time, 1);
      } else {
        if (speed instanceof Array) {
          message = Buffer.from([8, mapSpeed(speed[0]), mapSpeed(speed[1]), this._maxPower, this.useProfile()]);
        } else {
          message = Buffer.from([7, mapSpeed(speed), this._maxPower, this.useProfile()]);
        }
      }
      return this.sendPortOutputCommand(message, interrupt);
    }
    /**
     * Rotate a motor by a given amount of degrees.
     * @param {number} degrees How much the motor should be rotated (in degrees).
     * @param {number} [speed=100] For forward, a value between 1 - 100 should be set. For reverse, a value between -1 to -100.
     * @param {boolean} [interrupt=false] If true, previous commands are discarded.
     * @returns {Promise<CommandFeedback>} Resolved upon completion of command (i.e. once the motor is finished).
     */
    rotateByDegrees(degrees, speed, interrupt = false) {
      if (!this.isVirtualPort && speed instanceof Array) {
        throw new Error("Only virtual ports can accept multiple speeds");
      }
      if (this.isWeDo2SmartHub) {
        throw new Error("Rotation is not available on the WeDo 2.0 Smart Hub");
      }
      if (speed === void 0 || speed === null) {
        speed = 100;
      }
      let message;
      if (speed instanceof Array) {
        message = Buffer.from([12, 0, 0, 0, 0, mapSpeed(speed[0]), mapSpeed(speed[1]), this._maxPower, this._brakeStyle, this.useProfile()]);
      } else {
        message = Buffer.from([11, 0, 0, 0, 0, mapSpeed(speed), this._maxPower, this._brakeStyle, this.useProfile()]);
      }
      message.writeUInt32LE(degrees, 1);
      return this.sendPortOutputCommand(message, interrupt);
    }
    useProfile() {
      let value = 0;
      if (this.useAccelerationProfile) {
        value += 1;
      }
      if (this.useDecelerationProfile) {
        value += 2;
      }
      return value;
    }
  };
  var ModeMap8 = {
    "rotate": 2 /* ROTATION */
  };

  // src/devices/mediumlinearmotor.ts
  var MediumLinearMotor = class extends TachoMotor {
    constructor(hub, portId) {
      super(hub, portId, {}, 38 /* MEDIUM_LINEAR_MOTOR */);
    }
  };

  // src/devices/motionsensor.ts
  var MotionSensor = class extends Device {
    constructor(hub, portId) {
      super(hub, portId, ModeMap9, 35 /* MOTION_SENSOR */);
    }
    receive(message) {
      const mode = this._mode;
      switch (mode) {
        case 0 /* DISTANCE */:
          let distance = message[this.isWeDo2SmartHub ? 2 : 4];
          if (message[this.isWeDo2SmartHub ? 3 : 5] === 1) {
            distance = distance + 255;
          }
          distance *= 10;
          this.notify("distance", { distance });
          break;
      }
    }
  };
  var ModeMap9 = {
    "distance": 0 /* DISTANCE */
  };

  // src/devices/movehubmediumlinearmotor.ts
  var MoveHubMediumLinearMotor = class extends TachoMotor {
    constructor(hub, portId) {
      super(hub, portId, {}, 39 /* MOVE_HUB_MEDIUM_LINEAR_MOTOR */);
    }
  };

  // src/devices/movehubtiltsensor.ts
  var MoveHubTiltSensor = class extends Device {
    constructor(hub, portId) {
      super(hub, portId, ModeMap10, 40 /* MOVE_HUB_TILT_SENSOR */);
    }
    receive(message) {
      const mode = this._mode;
      switch (mode) {
        case 0 /* TILT */:
          const x = -message.readInt8(4);
          const y = message.readInt8(5);
          this.notify("tilt", { x, y });
          break;
      }
    }
  };
  var ModeMap10 = {
    "tilt": 0 /* TILT */
  };

  // src/devices/piezobuzzer.ts
  var PiezoBuzzer = class extends Device {
    constructor(hub, portId) {
      super(hub, portId, {}, 22 /* PIEZO_BUZZER */);
    }
    /**
     * Play a tone on the Hub's in-built buzzer
     * @param {number} frequency
     * @param {number} time How long the tone should play for (in milliseconds).
     * @returns {Promise<CommandFeedback>} Resolved upon completion of command (i.e. once the tone has finished playing).
     */
    playTone(frequency, time) {
      return new Promise((resolve) => {
        const data = Buffer.from([5, 2, 4, 0, 0, 0, 0]);
        data.writeUInt16LE(frequency, 3);
        data.writeUInt16LE(time, 5);
        this.send(data, "00001565-1212-efde-1523-785feabcd123" /* WEDO2_MOTOR_VALUE_WRITE */);
        globalThis.setTimeout(() => {
          return resolve(38 /* FEEDBACK_DISABLED */);
        }, time);
      });
    }
  };

  // src/devices/remotecontrolbutton.ts
  var RemoteControlButton = class extends Device {
    constructor(hub, portId) {
      super(hub, portId, ModeMap11, 55 /* REMOTE_CONTROL_BUTTON */);
    }
    receive(message) {
      const mode = this._mode;
      switch (mode) {
        case 0 /* BUTTON_EVENTS */:
          const event = message[4];
          this.notify("remoteButton", { event });
          break;
      }
    }
  };
  var ModeMap11 = {
    "remoteButton": 0 /* BUTTON_EVENTS */
  };

  // src/devices/simplemediumlinearmotor.ts
  var SimpleMediumLinearMotor = class extends BasicMotor {
    constructor(hub, portId) {
      super(hub, portId, {}, 1 /* SIMPLE_MEDIUM_LINEAR_MOTOR */);
    }
  };

  // src/devices/techniccolorsensor.ts
  var TechnicColorSensor = class extends Device {
    constructor(hub, portId) {
      super(hub, portId, ModeMap12, 61 /* TECHNIC_COLOR_SENSOR */);
    }
    receive(message) {
      const mode = this._mode;
      let hue;
      let saturation;
      let value;
      let intensity;
      switch (mode) {
        case 0 /* COLOR */:
          if (message.length !== 5) {
            break;
          }
          if (message[4] <= 10) {
            const color = parseColor(message[4]);
            this.notify("color", { color });
          }
          break;
        case 1 /* REFLECT */:
          if (message.length !== 5) {
            break;
          }
          const reflect = message[4];
          this.notify("reflect", { reflect });
          break;
        case 2 /* AMBIENT */:
          if (message.length !== 5) {
            break;
          }
          const ambient = message[4];
          this.notify("ambient", { ambient });
          break;
        case 5 /* RGB_I */:
          if (this.isWeDo2SmartHub) {
            break;
          }
          if (message.length !== 12) {
            break;
          }
          const red = message.readUInt16LE(4);
          const green = message.readUInt16LE(6);
          const blue = message.readUInt16LE(8);
          intensity = message.readUInt16LE(10);
          this.notify("rgbIntensity", { red, green, blue, intensity });
          break;
        case 6 /* HSV */:
          if (this.isWeDo2SmartHub) {
            break;
          }
          if (message.length !== 10) {
            break;
          }
          hue = message.readUInt16LE(4);
          saturation = message.readUInt16LE(6);
          value = message.readUInt16LE(8);
          this.notify("hsvIntensity", { hue, saturation, value });
          break;
        case 7 /* SHSV */:
          if (this.isWeDo2SmartHub) {
            break;
          }
          if (message.length !== 12) {
            break;
          }
          hue = message.readUInt16LE(4);
          saturation = message.readUInt16LE(6);
          value = message.readUInt16LE(8);
          intensity = message.readUInt16LE(10);
          this.notify("hsvAmbient", { hue, saturation, value, intensity });
          break;
      }
    }
    /**
     * Set the brightness (or turn on/off) of the lights around the sensor.
     * @param {number} firstSegment First light segment. 0-100 brightness.
     * @param {number} secondSegment Second light segment. 0-100 brightness.
     * @param {number} thirdSegment Third light segment. 0-100 brightness.
     * @returns {Promise<CommandFeedback>} Resolved upon completion of the command.
     */
    setBrightness(firstSegment, secondSegment, thirdSegment) {
      this.subscribe(3 /* LIGHT */);
      return this.writeDirect(3 /* LIGHT */, Buffer.from([firstSegment, secondSegment, thirdSegment]));
    }
  };
  var ModeMap12 = {
    "color": 0 /* COLOR */,
    "reflect": 1 /* REFLECT */,
    "ambient": 2 /* AMBIENT */,
    "rgbIntensity": 5 /* RGB_I */,
    "hsvIntensity": 6 /* HSV */,
    "hsvAmbient": 7 /* SHSV */
  };

  // src/devices/technicdistancesensor.ts
  var TechnicDistanceSensor = class extends Device {
    constructor(hub, portId) {
      super(hub, portId, ModeMap13, 62 /* TECHNIC_DISTANCE_SENSOR */);
    }
    receive(message) {
      const mode = this._mode;
      switch (mode) {
        case 0 /* DISTANCE */:
          const distance = message.readUInt16LE(4);
          this.notify("distance", { distance });
          break;
        case 1 /* FAST_DISTANCE */:
          const fastDistance = message.readUInt16LE(4);
          this.notify("fastDistance", { fastDistance });
          break;
      }
    }
    /**
     * Set the brightness (or turn on/off) of the lights around the eyes.
     * @param {number} topLeft Top left quadrant (above left eye). 0-100 brightness.
     * @param {number} bottomLeft Bottom left quadrant (below left eye). 0-100 brightness.
     * @param {number} topRight Top right quadrant (above right eye). 0-100 brightness.
     * @param {number} bottomRight Bottom right quadrant (below right eye). 0-100 brightness.
     * @returns {Promise<CommandFeedback>} Resolved upon completion of the command.
     */
    setBrightness(topLeft, bottomLeft, topRight, bottomRight) {
      this.writeDirect(5, Buffer.from([topLeft, topRight, bottomLeft, bottomRight]));
    }
  };
  var ModeMap13 = {
    "distance": 0 /* DISTANCE */,
    "fastDistance": 1 /* FAST_DISTANCE */
  };

  // src/devices/technicforcesensor.ts
  var TechnicForceSensor = class extends Device {
    constructor(hub, portId) {
      super(hub, portId, ModeMap14, 63 /* TECHNIC_FORCE_SENSOR */);
    }
    receive(message) {
      const mode = this._mode;
      switch (mode) {
        case 0 /* FORCE */:
          const force = message[this.isWeDo2SmartHub ? 2 : 4] / 10;
          this.notify("force", { force });
          break;
        case 1 /* TOUCHED */:
          const touched = message[4] ? true : false;
          this.notify("touched", { touched });
          break;
        case 2 /* TAPPED */:
          const tapped = message[4];
          this.notify("tapped", { tapped });
          break;
      }
    }
  };
  var ModeMap14 = {
    "force": 0 /* FORCE */,
    "touched": 1 /* TOUCHED */,
    "tapped": 2 /* TAPPED */
  };

  // src/devices/absolutemotor.ts
  var AbsoluteMotor = class extends TachoMotor {
    constructor(hub, portId, modeMap = {}, type = 0 /* UNKNOWN */) {
      super(hub, portId, Object.assign({}, modeMap, ModeMap15), type);
    }
    receive(message) {
      const mode = this._mode;
      switch (mode) {
        case 3 /* ABSOLUTE */:
          const angle = normalizeAngle(message.readInt16LE(this.isWeDo2SmartHub ? 2 : 4));
          this.notify("absolute", { angle });
          break;
        default:
          super.receive(message);
          break;
      }
    }
    /**
     * Rotate a motor by a given angle.
     * @param {number} angle Absolute position the motor should go to (degrees from 0).
     * @param {number} [speed=100] For forward, a value between 1 - 100 should be set. For reverse, a value between -1 to -100.
     * @param {boolean} [interrupt=false] If true, previous commands are discarded.
     * @returns {Promise<CommandFeedback>} Resolved upon completion of command (i.e. once the motor is finished).
     */
    gotoAngle(angle, speed = 100, interrupt = false) {
      if (!this.isVirtualPort && angle instanceof Array) {
        throw new Error("Only virtual ports can accept multiple positions");
      }
      if (this.isWeDo2SmartHub) {
        throw new Error("Absolute positioning is not available on the WeDo 2.0 Smart Hub");
      }
      if (speed === void 0 || speed === null) {
        speed = 100;
      }
      let message;
      if (angle instanceof Array) {
        message = Buffer.from([14, 0, 0, 0, 0, 0, 0, 0, 0, mapSpeed(speed), this._maxPower, this._brakeStyle, this.useProfile()]);
        message.writeInt32LE(normalizeAngle(angle[0]), 1);
        message.writeInt32LE(normalizeAngle(angle[1]), 5);
      } else {
        message = Buffer.from([13, 0, 0, 0, 0, mapSpeed(speed), this._maxPower, this._brakeStyle, this.useProfile()]);
        message.writeInt32LE(normalizeAngle(angle), 1);
      }
      return this.sendPortOutputCommand(message, interrupt);
    }
    /**
     * Rotate motor to real zero position.
     *
     * Real zero is marked on Technic angular motors (SPIKE Prime). It is also available on Technic linear motors (Control+) but is unmarked.
     * @param {number} [speed=100] Speed between 1 - 100. Note that this will always take the shortest path to zero.
     * @returns {Promise<CommandFeedback>} Resolved upon completion of command (i.e. once the motor is finished).
     */
    gotoRealZero(speed = 100) {
      return new Promise((resolve) => {
        const oldMode = this.mode;
        let calibrated = false;
        this.on("absolute", async ({ angle }) => {
          if (!calibrated) {
            calibrated = true;
            if (angle < 0) {
              angle = Math.abs(angle);
            } else {
              speed = -speed;
            }
            await this.rotateByDegrees(angle, speed);
            if (oldMode) {
              this.subscribe(oldMode);
            }
            return resolve(38 /* FEEDBACK_DISABLED */);
          }
        });
        this.requestUpdate();
      });
    }
    /**
     * Reset zero to current position
     * @param {boolean} [interrupt=false] If true, previous commands are discarded.
     * @returns {Promise<CommandFeedback>} Resolved upon completion of command (i.e. once the motor is finished).
     */
    resetZero(interrupt = false) {
      const data = Buffer.from([81, 2, 0, 0, 0, 0]);
      return this.sendPortOutputCommand(data, interrupt);
    }
  };
  var ModeMap15 = {
    "rotate": 2 /* ROTATION */,
    "absolute": 3 /* ABSOLUTE */
  };

  // src/devices/techniclargeangularmotor.ts
  var TechnicLargeAngularMotor = class extends AbsoluteMotor {
    constructor(hub, portId, modeMap = {}, type = 49 /* TECHNIC_LARGE_ANGULAR_MOTOR */) {
      super(hub, portId, {}, type);
    }
  };

  // src/devices/techniclargelinearmotor.ts
  var TechnicLargeLinearMotor = class extends AbsoluteMotor {
    constructor(hub, portId) {
      super(hub, portId, {}, 46 /* TECHNIC_LARGE_LINEAR_MOTOR */);
    }
  };

  // src/devices/technicsmallangularmotor.ts
  var TechnicSmallAngularMotor = class extends AbsoluteMotor {
    constructor(hub, portId, modeMap = {}, type = 65 /* TECHNIC_SMALL_ANGULAR_MOTOR */) {
      super(hub, portId, {}, type);
    }
  };

  // src/devices/technicmediumangularmotor.ts
  var TechnicMediumAngularMotor = class extends AbsoluteMotor {
    constructor(hub, portId, modeMap = {}, type = 48 /* TECHNIC_MEDIUM_ANGULAR_MOTOR */) {
      super(hub, portId, {}, type);
    }
  };

  // src/devices/technicmediumhubaccelerometersensor.ts
  var TechnicMediumHubAccelerometerSensor = class extends Device {
    constructor(hub, portId) {
      super(hub, portId, ModeMap16, 57 /* TECHNIC_MEDIUM_HUB_ACCELEROMETER */);
    }
    receive(message) {
      const mode = this._mode;
      switch (mode) {
        case 0 /* ACCEL */:
          const x = Math.round(message.readInt16LE(4) / 4.096);
          const y = Math.round(message.readInt16LE(6) / 4.096);
          const z = Math.round(message.readInt16LE(8) / 4.096);
          this.notify("accel", { x, y, z });
          break;
      }
    }
  };
  var ModeMap16 = {
    "accel": 0 /* ACCEL */
  };

  // src/devices/technicmediumhubgyrosensor.ts
  var TechnicMediumHubGyroSensor = class extends Device {
    constructor(hub, portId) {
      super(hub, portId, ModeMap17, 58 /* TECHNIC_MEDIUM_HUB_GYRO_SENSOR */);
    }
    receive(message) {
      const mode = this._mode;
      switch (mode) {
        case 0 /* GYRO */:
          const x = Math.round(message.readInt16LE(4) * 7 / 400);
          const y = Math.round(message.readInt16LE(6) * 7 / 400);
          const z = Math.round(message.readInt16LE(8) * 7 / 400);
          this.notify("gyro", { x, y, z });
          break;
      }
    }
  };
  var ModeMap17 = {
    "gyro": 0 /* GYRO */
  };

  // src/devices/technicmediumhubtiltsensor.ts
  var TechnicMediumHubTiltSensor = class extends Device {
    // guess of default value
    constructor(hub, portId) {
      super(hub, portId, ModeMap18, 59 /* TECHNIC_MEDIUM_HUB_TILT_SENSOR */);
      this._impactThreshold = 10;
      // guess of default value
      this._impactHoldoff = 10;
    }
    receive(message) {
      const mode = this._mode;
      switch (mode) {
        case 0 /* TILT */:
          let z = -message.readInt16LE(4);
          const y = message.readInt16LE(6);
          const x = message.readInt16LE(8);
          if (y === 90 || y === -90) {
            z = Math.sign(y) * (z + 180);
            if (z > 180) z -= 360;
            if (z < -180) z += 360;
          }
          this.notify("tilt", { x, y, z });
          break;
        case 1 /* IMPACT_COUNT */:
          if (message.length !== 8) {
            break;
          }
          const count = message.readUInt32LE(4);
          this.notify("impactCount", { count });
          break;
      }
    }
    /**
     * Set the impact count value.
     * @param {number} count impact count between 0 and 2^32
     * @returns {Promise<CommandFeedback>} Resolved upon completion of the command.
     */
    setImpactCount(count) {
      const payload = Buffer.alloc(4);
      payload.writeUInt32LE(count % 2 ** 32);
      return this.writeDirect(1, payload);
    }
    /**
     * Set the impact threshold.
     * @param {number} threshold value between 1 and 127
     * @returns {Promise<CommandFeedback>} Resolved upon completion of the command.
     */
    setImpactThreshold(threshold) {
      this._impactThreshold = threshold;
      return this.writeDirect(2, Buffer.from([this._impactThreshold, this._impactHoldoff]));
    }
    /**
     * Set the impact holdoff time.
     * @param {number} holdoff value between 1 and 127
     * @returns {Promise<CommandFeedback>} Resolved upon completion of the command.
     */
    setImpactHoldoff(holdoff) {
      this._impactHoldoff = holdoff;
      return this.writeDirect(2, Buffer.from([this._impactThreshold, this._impactHoldoff]));
    }
  };
  var ModeMap18 = {
    "tilt": 0 /* TILT */,
    "impactCount": 1 /* IMPACT_COUNT */
  };

  // src/devices/technicxlargelinearmotor.ts
  var TechnicXLargeLinearMotor = class extends AbsoluteMotor {
    constructor(hub, portId) {
      super(hub, portId, {}, 47 /* TECHNIC_XLARGE_LINEAR_MOTOR */);
    }
  };

  // src/devices/tiltsensor.ts
  var TiltSensor = class extends Device {
    constructor(hub, portId) {
      super(hub, portId, ModeMap19, 34 /* TILT_SENSOR */);
    }
    receive(message) {
      const mode = this._mode;
      let x = 0;
      let y = 0;
      let z = 0;
      switch (mode) {
        case 0 /* TILT */:
          if (message.length !== (this.isWeDo2SmartHub ? 4 : 6)) {
            break;
          }
          x = message.readInt8(this.isWeDo2SmartHub ? 2 : 4);
          y = message.readInt8(this.isWeDo2SmartHub ? 3 : 5);
          this.notify("tilt", { x, y });
          break;
        case 1 /* DIRECTION */:
          const dir = message.readInt8(this.isWeDo2SmartHub ? 2 : 4);
          this.notify("direction", { dir });
          break;
        case 2 /* CRASH */:
          if (message.length !== (this.isWeDo2SmartHub ? 5 : 7)) {
            break;
          }
          x = message.readUInt8(this.isWeDo2SmartHub ? 2 : 4);
          y = message.readUInt8(this.isWeDo2SmartHub ? 3 : 5);
          z = message.readUInt8(this.isWeDo2SmartHub ? 4 : 6);
          this.notify("impactCount", { x, y, z });
          break;
        case 3 /* CAL */:
          if (message.length !== (this.isWeDo2SmartHub ? 5 : 7)) {
            break;
          }
          const ax = message.readInt8(this.isWeDo2SmartHub ? 2 : 4);
          const ay = message.readInt8(this.isWeDo2SmartHub ? 3 : 5);
          const az = message.readInt8(this.isWeDo2SmartHub ? 4 : 6);
          const con = 1e3 / (45 * Math.sqrt(2));
          x = con * ax;
          y = con * ay;
          z = con * az;
          if (ax === 45) {
            x = con * Math.sqrt(2 * 45 ** 2 - ay ** 2 - az ** 2);
          } else if (ax === -45) {
            x = -con * Math.sqrt(2 * 45 ** 2 - ay ** 2 - az ** 2);
          }
          if (ay === 45) {
            y = con * Math.sqrt(2 * 45 ** 2 - ax ** 2 - az ** 2);
          } else if (ay === -45) {
            y = -con * Math.sqrt(2 * 45 ** 2 - ax ** 2 - az ** 2);
          }
          if (az === 45) {
            z = con * Math.sqrt(2 * 45 ** 2 - ax ** 2 - ay ** 2);
          } else if (az === -45) {
            z = -con * Math.sqrt(2 * 45 ** 2 - ax ** 2 - ay ** 2);
          }
          this.notify("accel", { x, y, z });
          break;
      }
    }
  };
  var ModeMap19 = {
    "tilt": 0 /* TILT */,
    "direction": 1 /* DIRECTION */,
    "impactCount": 2 /* CRASH */,
    "accel": 3 /* CAL */
  };

  // src/devices/trainmotor.ts
  var TrainMotor = class extends BasicMotor {
    constructor(hub, portId) {
      super(hub, portId, {}, 2 /* TRAIN_MOTOR */);
    }
  };

  // src/devices/voltagesensor.ts
  var VoltageSensor = class extends Device {
    constructor(hub, portId) {
      super(hub, portId, ModeMap20, 20 /* VOLTAGE_SENSOR */);
    }
    receive(message) {
      const mode = this._mode;
      switch (mode) {
        case 0 /* VOLTAGE */:
          if (this.isWeDo2SmartHub) {
            const voltage = message.readInt16LE(2) / 40;
            this.notify("voltage", { voltage });
          } else {
            let maxVoltageValue = MaxVoltageValue[this.hub.type];
            if (maxVoltageValue === void 0) {
              maxVoltageValue = MaxVoltageValue[0 /* UNKNOWN */];
            }
            let maxVoltageRaw = MaxVoltageRaw[this.hub.type];
            if (maxVoltageRaw === void 0) {
              maxVoltageRaw = MaxVoltageRaw[0 /* UNKNOWN */];
            }
            const voltage = message.readUInt16LE(4) * maxVoltageValue / maxVoltageRaw;
            this.notify("voltage", { voltage });
          }
          break;
      }
    }
  };
  var ModeMap20 = {
    "voltage": 0 /* VOLTAGE */
  };
  var MaxVoltageValue = {
    [0 /* UNKNOWN */]: 9.615,
    [5 /* DUPLO_TRAIN_BASE */]: 6.4,
    [4 /* REMOTE_CONTROL */]: 6.4
  };
  var MaxVoltageRaw = {
    [0 /* UNKNOWN */]: 3893,
    [5 /* DUPLO_TRAIN_BASE */]: 3047,
    [4 /* REMOTE_CONTROL */]: 3200,
    [6 /* TECHNIC_MEDIUM_HUB */]: 4095
  };

  // src/hubs/basehub.ts
  var import_debug2 = __toESM(require_browser(), 1);

  // src/color.ts
  var Color2 = class {
    constructor(color, brightness) {
      this._brightness = 100;
      this._color = color;
      this._brightness = brightness || 100;
    }
    toValue() {
      if (this._color === 255 /* NONE */) {
        return this._color;
      }
      return this._color + (Math.round(this._brightness / 10) << 4);
    }
  };

  // src/devices/technic3x3colorlightmatrix.ts
  var Technic3x3ColorLightMatrix = class extends Device {
    constructor(hub, portId) {
      super(hub, portId, {}, 64 /* TECHNIC_3X3_COLOR_LIGHT_MATRIX */);
    }
    /**
     * Set the LED matrix, one color per LED
     * @param {Color[] | Color} colors Array of 9 colors, 9 Color objects, or a single color
     * @returns {Promise<CommandFeedback>} Resolved upon completion of command.
     */
    setMatrix(colors) {
      this.subscribe(2 /* PIX_0 */);
      const colorArray = new Array(9);
      for (let i = 0; i < colorArray.length; i++) {
        if (typeof colors === "number") {
          colorArray[i] = colors + (10 << 4);
        }
        if (colors[i] instanceof Color2) {
          colorArray[i] = colors[i].toValue();
        }
        if (colors[i] === 255 /* NONE */) {
          colorArray[i] = 255 /* NONE */;
        }
        if (colors[i] <= 10) {
          colorArray[i] = colors[i] + (10 << 4);
        }
      }
      return this.writeDirect(2 /* PIX_0 */, Buffer.from(colorArray));
    }
  };

  // src/hubs/basehub.ts
  var debug4 = (0, import_debug2.default)("basehub");
  var BaseHub = class extends import_events3.EventEmitter {
    constructor(bleDevice, portMap = {}, type = 0 /* UNKNOWN */) {
      super();
      this._attachedDevices = {};
      this._name = "";
      this._firmwareVersion = "0.0.00.0000";
      this._hardwareVersion = "0.0.00.0000";
      this._primaryMACAddress = "00:00:00:00:00:00";
      this._batteryLevel = 100;
      this._rssi = -60;
      this._portMap = {};
      this._virtualPorts = [];
      this._attachCallbacks = [];
      this.setMaxListeners(23);
      this._type = type;
      this._bleDevice = bleDevice;
      this._portMap = Object.assign({}, portMap);
      bleDevice.on("disconnect", () => {
        this.emit("disconnect");
      });
    }
    /**
     * @readonly
     * @property {string} name Name of the hub
     */
    get name() {
      return this._bleDevice.name;
    }
    /**
     * @readonly
     * @property {string} connected Connected status
     */
    get connected() {
      return this._bleDevice.connected;
    }
    /**
     * @readonly
     * @property {string} connecting Connecting status
     */
    get connecting() {
      return this._bleDevice.connecting;
    }
    /**
     * @readonly
     * @property {string} type Hub type
     */
    get type() {
      return this._type;
    }
    /**
     * @readonly
     * @property {string[]} ports Array of port names
     */
    get ports() {
      return Object.keys(this._portMap);
    }
    /**
     * @readonly
     * @property {string} firmwareVersion Firmware version of the hub
     */
    get firmwareVersion() {
      return this._firmwareVersion;
    }
    /**
     * @readonly
     * @property {string} hardwareVersion Hardware version of the hub
     */
    get hardwareVersion() {
      return this._hardwareVersion;
    }
    /**
     * @readonly
     * @property {string} primaryMACAddress Primary MAC address of the hub
     */
    get primaryMACAddress() {
      return this._primaryMACAddress;
    }
    /**
     * @readonly
     * @property {string} uuid UUID of the hub
     */
    get uuid() {
      return this._bleDevice.uuid;
    }
    /**
     * @readonly
     * @property {number} batteryLevel Battery level of the hub (Percentage between 0-100)
     */
    get batteryLevel() {
      return this._batteryLevel;
    }
    /**
     * @readonly
     * @property {number} rssi Signal strength of the hub
     */
    get rssi() {
      return this._rssi;
    }
    /**
     * Connect to the Hub.
     * @returns {Promise} Resolved upon successful connect.
     */
    connect() {
      if (this._bleDevice.connecting) {
        throw new Error("Already connecting");
      } else if (this._bleDevice.connected) {
        throw new Error("Already connected");
      }
      return this._bleDevice.connect();
    }
    /**
     * Disconnect the Hub.
     * @returns {Promise} Resolved upon successful disconnect.
     */
    disconnect() {
      return this._bleDevice.disconnect();
    }
    /**
     * Retrieves the device attached to a given port.
     * @param {string} portName The name of the port to retrieve the device from.
     * @returns {Device | undefined} The device attached to the port.
     */
    getDeviceAtPort(portName) {
      const portId = this._portMap[portName];
      if (portId !== void 0) {
        return this._attachedDevices[portId];
      } else {
        return void 0;
      }
    }
    /**
     * Retrieves the device attached to a given port, waiting until one is attached if there isn't one.
     *
     * Note: If a device is never attached, the returned promise may never resolve.
     * @param {string} portName The name of the port to retrieve the device from.
     * @returns {Promise} Resolved once a device is attached, or resolved immediately if a device is already attached.
     */
    waitForDeviceAtPort(portName) {
      return new Promise((resolve) => {
        const existingDevice = this.getDeviceAtPort(portName);
        if (existingDevice) {
          return resolve(existingDevice);
        }
        this._attachCallbacks.push((device) => {
          if (device.portName === portName) {
            resolve(device);
            return true;
          } else {
            return false;
          }
        });
      });
    }
    /**
     * Retrieves all attached devices.
     * @returns {Device[]} Array of all attached devices.
     */
    getDevices() {
      return Object.values(this._attachedDevices);
    }
    /**
     * Retrieves an array of devices of the specified type.
     * @param {number} deviceType The device type to lookup.
     * @returns {Device[]} Array of all devices of the specified type.
     */
    getDevicesByType(deviceType) {
      return this.getDevices().filter((device) => device.type === deviceType);
    }
    /**
     * Retrieves the first device attached of the specified type, waiting until one is attached if there isn't one.
     *
     * Note: If a device is never attached, the returned promise may never resolve.
     * @param {number} deviceType The device type to lookup.
     * @returns {Promise} Resolved once a device is attached, or resolved immediately if a device is already attached.
     */
    waitForDeviceByType(deviceType) {
      return new Promise((resolve) => {
        const existingDevices = this.getDevicesByType(deviceType);
        if (existingDevices.length >= 1) {
          return resolve(existingDevices[0]);
        }
        this._attachCallbacks.push((device) => {
          if (device.type === deviceType) {
            resolve(device);
            return true;
          } else {
            return false;
          }
        });
      });
    }
    getPortNameForPortId(portId) {
      for (const port of Object.keys(this._portMap)) {
        if (this._portMap[port] === portId) {
          return port;
        }
      }
      return;
    }
    isPortVirtual(portId) {
      return this._virtualPorts.indexOf(portId) > -1;
    }
    /**
     * Sleep a given amount of time.
     *
     * Note: This is a helper method to make it easier to add delays into a chain of commands.
     * @param {number} delay How long to sleep (in milliseconds).
     * @returns {Promise} Resolved after the delay is finished.
     */
    sleep(delay) {
      return new Promise((resolve) => {
        globalThis.setTimeout(resolve, delay);
      });
    }
    /**
     * Wait until a given list of concurrently running commands are complete.
     *
     * Note: This is a helper method to make it easier to wait for concurrent commands to complete.
     * @param {Array<Promise<any>>} commands Array of executing commands.
     * @returns {Promise} Resolved after the commands are finished.
     */
    wait(commands) {
      return Promise.all(commands);
    }
    send(message, uuid) {
      return Promise.resolve();
    }
    subscribe(portId, deviceType, mode) {
    }
    unsubscribe(portId, deviceType, mode) {
    }
    manuallyAttachDevice(deviceType, portId) {
      if (!this._attachedDevices[portId]) {
        debug4(`No device attached to portId ${portId}, creating and attaching device type ${deviceType}`);
        const device = this._createDevice(deviceType, portId);
        this._attachDevice(device);
        return device;
      } else {
        if (this._attachedDevices[portId].type === deviceType) {
          debug4(`Device of ${deviceType} already attached to portId ${portId}, returning existing device`);
          return this._attachedDevices[portId];
        } else {
          throw new Error(`Already a different type of device attached to portId ${portId}. Only use this method when you are certain what's attached.`);
        }
      }
    }
    manuallyDetachDevice(portId) {
      const device = this._attachedDevices[portId];
      if (!device) {
        debug4(`No device attached to portId ${portId}, nothing to detach`);
        return;
      }
      debug4(`Detaching device type ${device.type} from portId ${portId}`);
      this._detachDevice(device);
      if (this.isPortVirtual(portId)) {
        const portName = this.getPortNameForPortId(portId);
        if (portName) {
          delete this._portMap[portName];
        }
        this._virtualPorts = this._virtualPorts.filter((virtualPortId) => virtualPortId !== portId);
      }
      return device;
    }
    _attachDevice(device) {
      if (this._attachedDevices[device.portId] && this._attachedDevices[device.portId].type === device.type) {
        return;
      }
      this._attachedDevices[device.portId] = device;
      this.emit("attach", device);
      debug4(`Attached device type ${device.type} (${DeviceTypeNames[device.type]}) on port ${device.portName} (${device.portId})`);
      let i = this._attachCallbacks.length;
      while (i--) {
        const callback = this._attachCallbacks[i];
        if (callback(device)) {
          this._attachCallbacks.splice(i, 1);
        }
      }
    }
    _detachDevice(device) {
      delete this._attachedDevices[device.portId];
      this.emit("detach", device);
      debug4(`Detached device type ${device.type} (${DeviceTypeNames[device.type]}) on port ${device.portName} (${device.portId})`);
    }
    _createDevice(deviceType, portId) {
      let constructor;
      const deviceConstructors = {
        [8 /* LIGHT */]: Light,
        [2 /* TRAIN_MOTOR */]: TrainMotor,
        [1 /* SIMPLE_MEDIUM_LINEAR_MOTOR */]: SimpleMediumLinearMotor,
        [39 /* MOVE_HUB_MEDIUM_LINEAR_MOTOR */]: MoveHubMediumLinearMotor,
        [35 /* MOTION_SENSOR */]: MotionSensor,
        [34 /* TILT_SENSOR */]: TiltSensor,
        [40 /* MOVE_HUB_TILT_SENSOR */]: MoveHubTiltSensor,
        [22 /* PIEZO_BUZZER */]: PiezoBuzzer,
        [61 /* TECHNIC_COLOR_SENSOR */]: TechnicColorSensor,
        [62 /* TECHNIC_DISTANCE_SENSOR */]: TechnicDistanceSensor,
        [63 /* TECHNIC_FORCE_SENSOR */]: TechnicForceSensor,
        [59 /* TECHNIC_MEDIUM_HUB_TILT_SENSOR */]: TechnicMediumHubTiltSensor,
        [58 /* TECHNIC_MEDIUM_HUB_GYRO_SENSOR */]: TechnicMediumHubGyroSensor,
        [57 /* TECHNIC_MEDIUM_HUB_ACCELEROMETER */]: TechnicMediumHubAccelerometerSensor,
        [38 /* MEDIUM_LINEAR_MOTOR */]: MediumLinearMotor,
        [65 /* TECHNIC_SMALL_ANGULAR_MOTOR */]: TechnicSmallAngularMotor,
        [48 /* TECHNIC_MEDIUM_ANGULAR_MOTOR */]: TechnicMediumAngularMotor,
        [49 /* TECHNIC_LARGE_ANGULAR_MOTOR */]: TechnicLargeAngularMotor,
        [46 /* TECHNIC_LARGE_LINEAR_MOTOR */]: TechnicLargeLinearMotor,
        [47 /* TECHNIC_XLARGE_LINEAR_MOTOR */]: TechnicXLargeLinearMotor,
        [37 /* COLOR_DISTANCE_SENSOR */]: ColorDistanceSensor,
        [20 /* VOLTAGE_SENSOR */]: VoltageSensor,
        [21 /* CURRENT_SENSOR */]: CurrentSensor,
        [55 /* REMOTE_CONTROL_BUTTON */]: RemoteControlButton,
        [23 /* HUB_LED */]: HubLED,
        [43 /* DUPLO_TRAIN_BASE_COLOR_SENSOR */]: DuploTrainBaseColorSensor,
        [41 /* DUPLO_TRAIN_BASE_MOTOR */]: DuploTrainBaseMotor,
        [42 /* DUPLO_TRAIN_BASE_SPEAKER */]: DuploTrainBaseSpeaker,
        [44 /* DUPLO_TRAIN_BASE_SPEEDOMETER */]: DuploTrainBaseSpeedometer,
        [71 /* MARIO_ACCELEROMETER */]: MarioAccelerometer,
        [73 /* MARIO_BARCODE_SENSOR */]: MarioBarcodeSensor,
        [74 /* MARIO_PANTS_SENSOR */]: MarioPantsSensor,
        [75 /* TECHNIC_MEDIUM_ANGULAR_MOTOR_GREY */]: TechnicMediumAngularMotor,
        [76 /* TECHNIC_LARGE_ANGULAR_MOTOR_GREY */]: TechnicLargeAngularMotor,
        [64 /* TECHNIC_3X3_COLOR_LIGHT_MATRIX */]: Technic3x3ColorLightMatrix
      };
      constructor = deviceConstructors[deviceType];
      if (constructor) {
        return new constructor(this, portId, void 0, deviceType);
      } else {
        return new Device(this, portId, void 0, deviceType);
      }
    }
    _getDeviceByPortId(portId) {
      return this._attachedDevices[portId];
    }
  };

  // src/hubs/lpf2hub.ts
  var import_debug3 = __toESM(require_browser(), 1);
  var debug5 = (0, import_debug3.default)("lpf2hub");
  var modeInfoDebug = (0, import_debug3.default)("lpf2hubmodeinfo");
  var LPF2Hub = class extends BaseHub {
    constructor() {
      super(...arguments);
      this._messageBuffer = Buffer.alloc(0);
      this._propertyRequestCallbacks = {};
    }
    async connect() {
      debug5("LPF2Hub connecting");
      await super.connect();
      await this._bleDevice.discoverCharacteristicsForService("00001623-1212-efde-1623-785feabcd123" /* LPF2_HUB */);
      this._bleDevice.subscribeToCharacteristic("00001624-1212-efde-1623-785feabcd123" /* LPF2_ALL */, this._parseMessage.bind(this));
      await this._requestHubPropertyReports(2 /* BUTTON_STATE */);
      await this._requestHubPropertyValue(3 /* FW_VERSION */);
      await this._requestHubPropertyValue(4 /* HW_VERSION */);
      await this._requestHubPropertyReports(5 /* RSSI */);
      await this._requestHubPropertyReports(6 /* BATTERY_VOLTAGE */);
      await this._requestHubPropertyValue(13 /* PRIMARY_MAC_ADDRESS */);
      this.emit("connect");
      debug5("LPF2Hub connected");
    }
    /**
     * Shutdown the Hub.
     * @returns {Promise} Resolved upon successful disconnect.
     */
    shutdown() {
      return this.send(Buffer.from([2, 1]), "00001624-1212-efde-1623-785feabcd123" /* LPF2_ALL */);
    }
    /**
     * Set the name of the Hub.
     * @param {string} name New name of the hub (14 characters or less, ASCII only).
     * @returns {Promise} Resolved upon successful issuance of command.
     */
    async setName(name) {
      if (name.length > 14) {
        throw new Error("Name must be 14 characters or less");
      }
      let data = Buffer.from([1, 1, 1]);
      data = Buffer.concat([data, Buffer.from(name, "ascii")]);
      await this.send(data, "00001624-1212-efde-1623-785feabcd123" /* LPF2_ALL */);
      await this.send(data, "00001624-1212-efde-1623-785feabcd123" /* LPF2_ALL */);
      this._name = name;
    }
    send(message, uuid) {
      message = Buffer.concat([Buffer.alloc(2), message]);
      message[0] = message.length;
      debug5("Sent Message (LPF2_ALL)", message);
      return this._bleDevice.writeToCharacteristic(uuid, message);
    }
    subscribe(portId, deviceType, mode) {
      return this.send(Buffer.from([65, portId, mode, 1, 0, 0, 0, 1]), "00001624-1212-efde-1623-785feabcd123" /* LPF2_ALL */);
    }
    unsubscribe(portId, mode) {
      return this.send(Buffer.from([65, portId, mode, 1, 0, 0, 0, 0]), "00001624-1212-efde-1623-785feabcd123" /* LPF2_ALL */);
    }
    /**
     * Combines two ports with into a single virtual port.
     *
     * Note: The devices attached to the ports must be of the same device type.
     * @param {string} firstPortName First port name
     * @param {string} secondPortName Second port name
     * @returns {Promise} Resolved upon successful issuance of command.
     */
    createVirtualPort(firstPortName, secondPortName) {
      const firstDevice = this.getDeviceAtPort(firstPortName);
      if (!firstDevice) {
        throw new Error(`Port ${firstPortName} does not have an attached device`);
      }
      const secondDevice = this.getDeviceAtPort(secondPortName);
      if (!secondDevice) {
        throw new Error(`Port ${secondPortName} does not have an attached device`);
      }
      if (firstDevice.type !== secondDevice.type) {
        throw new Error(`Both devices must be of the same type to create a virtual port`);
      }
      return this.send(Buffer.from([97, 1, firstDevice.portId, secondDevice.portId]), "00001624-1212-efde-1623-785feabcd123" /* LPF2_ALL */);
    }
    _checkFirmware(version) {
      return;
    }
    _parseMessage(data) {
      if (data) {
        this._messageBuffer = Buffer.concat([this._messageBuffer, data]);
      }
      if (this._messageBuffer.length <= 0) {
        return;
      }
      const len = this._messageBuffer[0];
      if (len <= this._messageBuffer.length) {
        const message = this._messageBuffer.slice(0, len);
        this._messageBuffer = this._messageBuffer.slice(len);
        debug5("Received Message (LPF2_ALL)", message);
        switch (message[2]) {
          case 1 /* HUB_PROPERTIES */: {
            const property = message[3];
            const callback = this._propertyRequestCallbacks[property];
            if (callback) {
              callback(message);
            } else {
              this._parseHubPropertyResponse(message);
            }
            delete this._propertyRequestCallbacks[property];
            break;
          }
          case 4 /* HUB_ATTACHED_IO */: {
            this._parsePortMessage(message);
            break;
          }
          case 67 /* PORT_INFORMATION */: {
            this._parsePortInformationResponse(message);
            break;
          }
          case 68 /* PORT_MODE_INFORMATION */: {
            this._parseModeInformationResponse(message);
            break;
          }
          case 69 /* PORT_VALUE_SINGLE */: {
            this._parseSensorMessage(message);
            break;
          }
          case 71 /* PORT_INPUT_FORMAT_SINGLE */: {
            this._parsePortInputFormatMessage(message);
            break;
          }
          case 130 /* PORT_OUTPUT_COMMAND_FEEDBACK */: {
            this._parsePortAction(message);
            break;
          }
        }
        if (this._messageBuffer.length > 0) {
          this._parseMessage();
        }
      }
    }
    _requestHubPropertyValue(property) {
      return new Promise((resolve) => {
        this._propertyRequestCallbacks[property] = (message) => {
          this._parseHubPropertyResponse(message);
          return resolve();
        };
        this.send(Buffer.from([1, property, 5]), "00001624-1212-efde-1623-785feabcd123" /* LPF2_ALL */);
      });
    }
    _requestHubPropertyReports(property) {
      return this.send(Buffer.from([1, property, 2]), "00001624-1212-efde-1623-785feabcd123" /* LPF2_ALL */);
    }
    _parseHubPropertyResponse(message) {
      if (message[3] === 2 /* BUTTON_STATE */) {
        if (message[5] === 1) {
          this.emit("button", { event: 2 /* PRESSED */ });
          return;
        } else if (message[5] === 0) {
          this.emit("button", { event: 0 /* RELEASED */ });
          return;
        }
      } else if (message[3] === 3 /* FW_VERSION */) {
        this._firmwareVersion = decodeVersion(message.readInt32LE(5));
        this._checkFirmware(this._firmwareVersion);
      } else if (message[3] === 4 /* HW_VERSION */) {
        this._hardwareVersion = decodeVersion(message.readInt32LE(5));
      } else if (message[3] === 5 /* RSSI */) {
        const rssi = message.readInt8(5);
        if (rssi !== 0) {
          this._rssi = rssi;
          this.emit("rssi", { rssi: this._rssi });
        }
      } else if (message[3] === 13 /* PRIMARY_MAC_ADDRESS */) {
        this._primaryMACAddress = decodeMACAddress(message.slice(5));
      } else if (message[3] === 6 /* BATTERY_VOLTAGE */) {
        const batteryLevel = message[5];
        if (batteryLevel !== this._batteryLevel) {
          this._batteryLevel = batteryLevel;
          this.emit("batteryLevel", { batteryLevel });
        }
      }
    }
    async _parsePortMessage(message) {
      const portId = message[3];
      const event = message[4];
      const deviceType = event ? message.readUInt16LE(5) : 0;
      if (event === 1 /* ATTACHED_IO */) {
        if (modeInfoDebug.enabled) {
          const deviceTypeName = DeviceTypeNames[message[5]] || "Unknown";
          modeInfoDebug(`Port ${toHex(portId)}, type ${toHex(deviceType, 4)} (${deviceTypeName})`);
          const hwVersion = decodeVersion(message.readInt32LE(7));
          const swVersion = decodeVersion(message.readInt32LE(11));
          modeInfoDebug(`Port ${toHex(portId)}, hardware version ${hwVersion}, software version ${swVersion}`);
          await this._sendPortInformationRequest(portId);
        }
        const device = this._createDevice(deviceType, portId);
        this._attachDevice(device);
      } else if (event === 0 /* DETACHED_IO */) {
        const device = this._getDeviceByPortId(portId);
        if (device) {
          this._detachDevice(device);
          if (this.isPortVirtual(portId)) {
            const portName = this.getPortNameForPortId(portId);
            if (portName) {
              delete this._portMap[portName];
            }
            this._virtualPorts = this._virtualPorts.filter((virtualPortId) => virtualPortId !== portId);
          }
        }
      } else if (event === 2 /* ATTACHED_VIRTUAL_IO */) {
        const firstPortName = this.getPortNameForPortId(message[7]);
        const secondPortName = this.getPortNameForPortId(message[8]);
        const virtualPortName = firstPortName + secondPortName;
        const virtualPortId = message[3];
        this._portMap[virtualPortName] = virtualPortId;
        this._virtualPorts.push(virtualPortId);
        const device = this._createDevice(deviceType, virtualPortId);
        this._attachDevice(device);
      }
    }
    async _sendPortInformationRequest(port) {
      await this.send(Buffer.from([33, port, 1]), "00001624-1212-efde-1623-785feabcd123" /* LPF2_ALL */);
      await this.send(Buffer.from([33, port, 2]), "00001624-1212-efde-1623-785feabcd123" /* LPF2_ALL */);
    }
    async _parsePortInformationResponse(message) {
      const port = message[3];
      if (message[4] === 2) {
        const modeCombinationMasks = [];
        for (let i = 5; i < message.length; i += 2) {
          modeCombinationMasks.push(message.readUInt16LE(i));
        }
        modeInfoDebug(`Port ${toHex(port)}, mode combinations [${modeCombinationMasks.map((c) => toBin(c, 0)).join(", ")}]`);
        return;
      }
      const count = message[6];
      const input = toBin(message.readUInt16LE(7), count);
      const output = toBin(message.readUInt16LE(9), count);
      modeInfoDebug(`Port ${toHex(port)}, total modes ${count}, input modes ${input}, output modes ${output}`);
      for (let i = 0; i < count; i++) {
        await this._sendModeInformationRequest(port, i, 0 /* NAME */);
        await this._sendModeInformationRequest(port, i, 1 /* RAW */);
        await this._sendModeInformationRequest(port, i, 2 /* PCT */);
        await this._sendModeInformationRequest(port, i, 3 /* SI */);
        await this._sendModeInformationRequest(port, i, 4 /* SYMBOL */);
        await this._sendModeInformationRequest(port, i, 128 /* VALUE_FORMAT */);
      }
    }
    _sendModeInformationRequest(port, mode, type) {
      return this.send(Buffer.from([34, port, mode, type]), "00001624-1212-efde-1623-785feabcd123" /* LPF2_ALL */);
    }
    _parseModeInformationResponse(message) {
      const port = toHex(message[3]);
      const mode = message[4];
      const type = message[5];
      switch (type) {
        case 0 /* NAME */:
          modeInfoDebug(`Port ${port}, mode ${mode}, name ${message.slice(6, message.length).toString()}`);
          break;
        case 1 /* RAW */:
          modeInfoDebug(`Port ${port}, mode ${mode}, RAW min ${message.readFloatLE(6)}, max ${message.readFloatLE(10)}`);
          break;
        case 2 /* PCT */:
          modeInfoDebug(`Port ${port}, mode ${mode}, PCT min ${message.readFloatLE(6)}, max ${message.readFloatLE(10)}`);
          break;
        case 3 /* SI */:
          modeInfoDebug(`Port ${port}, mode ${mode}, SI min ${message.readFloatLE(6)}, max ${message.readFloatLE(10)}`);
          break;
        case 4 /* SYMBOL */:
          modeInfoDebug(`Port ${port}, mode ${mode}, SI symbol ${message.slice(6, message.length).toString()}`);
          break;
        case 128 /* VALUE_FORMAT */:
          const numValues = message[6];
          const dataType = ["8bit", "16bit", "32bit", "float"][message[7]];
          const totalFigures = message[8];
          const decimals = message[9];
          modeInfoDebug(`Port ${port}, mode ${mode}, Value ${numValues} x ${dataType}, Decimal format ${totalFigures}.${decimals}`);
      }
    }
    _parsePortAction(message) {
      for (let offset = 3; offset < message.length; offset += 2) {
        const device = this._getDeviceByPortId(message[offset]);
        if (device) {
          device.finish(message[offset + 1]);
        }
      }
    }
    _parseSensorMessage(message) {
      const portId = message[3];
      const device = this._getDeviceByPortId(portId);
      if (device) {
        device.receive(message);
      }
    }
    _parsePortInputFormatMessage(message) {
      const portId = message[3];
      const device = this._getDeviceByPortId(portId);
      if (device) {
        device.setMode(message[4]);
      }
    }
  };

  // src/hubs/duplotrainbase.ts
  var import_debug4 = __toESM(require_browser(), 1);
  var debug6 = (0, import_debug4.default)("duplotrainbase");
  var DuploTrainBase = class extends LPF2Hub {
    static IsDuploTrainBase(peripheral) {
      return peripheral.advertisement && peripheral.advertisement.serviceUuids && peripheral.advertisement.serviceUuids.indexOf("00001623-1212-efde-1623-785feabcd123" /* LPF2_HUB */.replace(/-/g, "")) >= 0 && peripheral.advertisement.manufacturerData && peripheral.advertisement.manufacturerData.length > 3 && peripheral.advertisement.manufacturerData[3] === 32 /* DUPLO_TRAIN_BASE_ID */;
    }
    constructor(device) {
      super(device, PortMap, 5 /* DUPLO_TRAIN_BASE */);
      debug6("Discovered Duplo Train Base");
    }
    async connect() {
      debug6("Connecting to Duplo Train Base");
      await super.connect();
      debug6("Connect completed");
    }
  };
  var PortMap = {
    "MOTOR": 0,
    "COLOR": 18,
    "SPEEDOMETER": 19
  };

  // node_modules/compare-versions/lib/esm/utils.js
  var semver = /^[v^~<>=]*?(\d+)(?:\.([x*]|\d+)(?:\.([x*]|\d+)(?:\.([x*]|\d+))?(?:-([\da-z\-]+(?:\.[\da-z\-]+)*))?(?:\+[\da-z\-]+(?:\.[\da-z\-]+)*)?)?)?$/i;
  var validateAndParse = (version) => {
    if (typeof version !== "string") {
      throw new TypeError("Invalid argument expected string");
    }
    const match = version.match(semver);
    if (!match) {
      throw new Error(`Invalid argument not valid semver ('${version}' received)`);
    }
    match.shift();
    return match;
  };
  var isWildcard = (s) => s === "*" || s === "x" || s === "X";
  var tryParse = (v) => {
    const n = parseInt(v, 10);
    return isNaN(n) ? v : n;
  };
  var forceType = (a, b) => typeof a !== typeof b ? [String(a), String(b)] : [a, b];
  var compareStrings = (a, b) => {
    if (isWildcard(a) || isWildcard(b))
      return 0;
    const [ap, bp] = forceType(tryParse(a), tryParse(b));
    if (ap > bp)
      return 1;
    if (ap < bp)
      return -1;
    return 0;
  };
  var compareSegments = (a, b) => {
    for (let i = 0; i < Math.max(a.length, b.length); i++) {
      const r = compareStrings(a[i] || "0", b[i] || "0");
      if (r !== 0)
        return r;
    }
    return 0;
  };

  // node_modules/compare-versions/lib/esm/compareVersions.js
  var compareVersions = (v1, v2) => {
    const n1 = validateAndParse(v1);
    const n2 = validateAndParse(v2);
    const p1 = n1.pop();
    const p2 = n2.pop();
    const r = compareSegments(n1, n2);
    if (r !== 0)
      return r;
    if (p1 && p2) {
      return compareSegments(p1.split("."), p2.split("."));
    } else if (p1 || p2) {
      return p1 ? -1 : 1;
    }
    return 0;
  };

  // src/hubs/hub.ts
  var import_debug5 = __toESM(require_browser(), 1);
  var debug7 = (0, import_debug5.default)("hub");
  var Hub = class extends LPF2Hub {
    constructor(device) {
      super(device, PortMap2, 3 /* HUB */);
      this._currentPort = 59;
      debug7("Discovered Powered UP Hub");
    }
    static IsHub(peripheral) {
      return peripheral.advertisement && peripheral.advertisement.serviceUuids && peripheral.advertisement.serviceUuids.indexOf("00001623-1212-efde-1623-785feabcd123" /* LPF2_HUB */.replace(/-/g, "")) >= 0 && peripheral.advertisement.manufacturerData && peripheral.advertisement.manufacturerData.length > 3 && peripheral.advertisement.manufacturerData[3] === 65 /* HUB_ID */;
    }
    async connect() {
      debug7("Connecting to Powered UP Hub");
      await super.connect();
      debug7("Connect completed");
    }
    _checkFirmware(version) {
      if (compareVersions("1.1.00.0004", version) === 1) {
        throw new Error(`Your Powered Up Hub's (${this.name}) firmware is out of date and unsupported by this library. Please update it via the official Powered Up app.`);
      }
    }
  };
  var PortMap2 = {
    "A": 0,
    "B": 1,
    "HUB_LED": 50,
    "CURRENT_SENSOR": 59,
    "VOLTAGE_SENSOR": 60
  };

  // src/hubs/mario.ts
  var import_debug6 = __toESM(require_browser(), 1);
  var debug8 = (0, import_debug6.default)("movehub");
  var Mario = class extends LPF2Hub {
    static IsMario(peripheral) {
      return peripheral.advertisement && peripheral.advertisement.serviceUuids && peripheral.advertisement.serviceUuids.indexOf("00001623-1212-efde-1623-785feabcd123" /* LPF2_HUB */.replace(/-/g, "")) >= 0 && peripheral.advertisement.manufacturerData && peripheral.advertisement.manufacturerData.length > 3 && peripheral.advertisement.manufacturerData[3] === 67 /* MARIO_ID */;
    }
    constructor(device) {
      super(device, PortMap3, 7 /* MARIO */);
      debug8("Discovered Mario");
    }
    async connect() {
      debug8("Connecting to Mario");
      await super.connect();
      debug8("Connect completed");
    }
  };
  var PortMap3 = {};

  // src/hubs/movehub.ts
  var import_debug7 = __toESM(require_browser(), 1);
  var debug9 = (0, import_debug7.default)("movehub");
  var MoveHub = class extends LPF2Hub {
    static IsMoveHub(peripheral) {
      return peripheral.advertisement && peripheral.advertisement.serviceUuids && peripheral.advertisement.serviceUuids.indexOf("00001623-1212-efde-1623-785feabcd123" /* LPF2_HUB */.replace(/-/g, "")) >= 0 && peripheral.advertisement.manufacturerData && peripheral.advertisement.manufacturerData.length > 3 && peripheral.advertisement.manufacturerData[3] === 64 /* MOVE_HUB_ID */;
    }
    constructor(device) {
      super(device, PortMap4, 2 /* MOVE_HUB */);
      debug9("Discovered Move Hub");
    }
    async connect() {
      debug9("Connecting to Move Hub");
      await super.connect();
      debug9("Connect completed");
    }
    _checkFirmware(version) {
      if (compareVersions("2.0.00.0017", version) === 1) {
        throw new Error(`Your Move Hub's (${this.name}) firmware is out of date and unsupported by this library. Please update it via the official Powered Up app.`);
      }
    }
  };
  var PortMap4 = {
    "A": 0,
    "B": 1,
    "C": 2,
    "D": 3,
    "HUB_LED": 50,
    "TILT_SENSOR": 58,
    "CURRENT_SENSOR": 59,
    "VOLTAGE_SENSOR": 60
  };

  // src/hubs/remotecontrol.ts
  var import_debug8 = __toESM(require_browser(), 1);
  var debug10 = (0, import_debug8.default)("remotecontrol");
  var RemoteControl = class extends LPF2Hub {
    static IsRemoteControl(peripheral) {
      return peripheral.advertisement && peripheral.advertisement.serviceUuids && peripheral.advertisement.serviceUuids.indexOf("00001623-1212-efde-1623-785feabcd123" /* LPF2_HUB */.replace(/-/g, "")) >= 0 && peripheral.advertisement.manufacturerData && peripheral.advertisement.manufacturerData.length > 3 && peripheral.advertisement.manufacturerData[3] === 66 /* REMOTE_CONTROL_ID */;
    }
    constructor(device) {
      super(device, PortMap5, 4 /* REMOTE_CONTROL */);
      debug10("Discovered Powered UP Remote");
    }
    async connect() {
      debug10("Connecting to Powered UP Remote");
      await super.connect();
      debug10("Connect completed");
    }
  };
  var PortMap5 = {
    "LEFT": 0,
    "RIGHT": 1,
    "HUB_LED": 52,
    "VOLTAGE_SENSOR": 59,
    "REMOTE_CONTROL_RSSI": 60
  };

  // src/hubs/technicmediumhub.ts
  var import_debug9 = __toESM(require_browser(), 1);
  var debug11 = (0, import_debug9.default)("technicmediumhub");
  var TechnicMediumHub = class extends LPF2Hub {
    static IsTechnicMediumHub(peripheral) {
      return peripheral.advertisement && peripheral.advertisement.serviceUuids && peripheral.advertisement.serviceUuids.indexOf("00001623-1212-efde-1623-785feabcd123" /* LPF2_HUB */.replace(/-/g, "")) >= 0 && peripheral.advertisement.manufacturerData && peripheral.advertisement.manufacturerData.length > 3 && peripheral.advertisement.manufacturerData[3] === 128 /* TECHNIC_MEDIUM_HUB_ID */;
    }
    constructor(device) {
      super(device, PortMap6, 6 /* TECHNIC_MEDIUM_HUB */);
      debug11("Discovered Control+ Hub");
    }
    async connect() {
      debug11("Connecting to Control+ Hub");
      await super.connect();
      debug11("Connect completed");
    }
  };
  var PortMap6 = {
    "A": 0,
    "B": 1,
    "C": 2,
    "D": 3,
    "HUB_LED": 50,
    "CURRENT_SENSOR": 59,
    "VOLTAGE_SENSOR": 60,
    "ACCELEROMETER": 97,
    "GYRO_SENSOR": 98,
    "TILT_SENSOR": 99
  };

  // src/hubs/wedo2smarthub.ts
  var import_debug10 = __toESM(require_browser(), 1);
  var debug12 = (0, import_debug10.default)("wedo2smarthub");
  var WeDo2SmartHub = class extends BaseHub {
    constructor(device) {
      super(device, PortMap7, 1 /* WEDO2_SMART_HUB */);
      this._lastTiltX = 0;
      this._lastTiltY = 0;
      debug12("Discovered WeDo 2.0 Smart Hub");
    }
    static IsWeDo2SmartHub(peripheral) {
      return peripheral.advertisement && peripheral.advertisement.serviceUuids && peripheral.advertisement.serviceUuids.indexOf("00001523-1212-efde-1523-785feabcd123" /* WEDO2_SMART_HUB */.replace(/-/g, "")) >= 0;
    }
    connect() {
      return new Promise(async (resolve) => {
        debug12("Connecting to WeDo 2.0 Smart Hub");
        await super.connect();
        await this._bleDevice.discoverCharacteristicsForService("00001523-1212-efde-1523-785feabcd123" /* WEDO2_SMART_HUB */);
        await this._bleDevice.discoverCharacteristicsForService("00004f0e-1212-efde-1523-785feabcd123" /* WEDO2_SMART_HUB_2 */);
        if (!isWebBluetooth) {
          await this._bleDevice.discoverCharacteristicsForService("2a19" /* WEDO2_SMART_HUB_3 */);
          await this._bleDevice.discoverCharacteristicsForService("180f" /* WEDO2_SMART_HUB_4 */);
          await this._bleDevice.discoverCharacteristicsForService("180a" /* WEDO2_SMART_HUB_5 */);
        } else {
          await this._bleDevice.discoverCharacteristicsForService("battery_service");
          await this._bleDevice.discoverCharacteristicsForService("device_information");
        }
        debug12("Connect completed");
        this.emit("connect");
        resolve();
        this._bleDevice.subscribeToCharacteristic("00001527-1212-efde-1523-785feabcd123" /* WEDO2_PORT_TYPE */, this._parsePortMessage.bind(this));
        this._bleDevice.subscribeToCharacteristic("00001560-1212-efde-1523-785feabcd123" /* WEDO2_SENSOR_VALUE */, this._parseSensorMessage.bind(this));
        this._bleDevice.subscribeToCharacteristic("00001526-1212-efde-1523-785feabcd123" /* WEDO2_BUTTON */, this._parseSensorMessage.bind(this));
        if (!isWebBluetooth) {
          this._bleDevice.subscribeToCharacteristic("2a19" /* WEDO2_BATTERY */, this._parseBatteryMessage.bind(this));
          this._bleDevice.readFromCharacteristic("2a19" /* WEDO2_BATTERY */, (err, data) => {
            if (data) {
              this._parseBatteryMessage(data);
            }
          });
        } else {
          this._bleDevice.readFromCharacteristic("00002a19-0000-1000-8000-00805f9b34fb", (err, data) => {
            if (data) {
              this._parseBatteryMessage(data);
            }
          });
          this._bleDevice.subscribeToCharacteristic("00002a19-0000-1000-8000-00805f9b34fb", this._parseHighCurrentAlert.bind(this));
        }
        this._bleDevice.subscribeToCharacteristic("00001529-1212-efde-1523-785feabcd123" /* WEDO2_HIGH_CURRENT_ALERT */, this._parseHighCurrentAlert.bind(this));
        if (!isWebBluetooth) {
          this._bleDevice.readFromCharacteristic("2a26" /* WEDO2_FIRMWARE_REVISION */, (err, data) => {
            if (data) {
              this._parseFirmwareRevisionString(data);
            }
          });
        } else {
          this._bleDevice.readFromCharacteristic("00002a26-0000-1000-8000-00805f9b34fb", (err, data) => {
            if (data) {
              this._parseFirmwareRevisionString(data);
            }
          });
        }
      });
    }
    /**
     * Shutdown the Hub.
     * @returns {Promise} Resolved upon successful disconnect.
     */
    shutdown() {
      return this.send(Buffer.from([0]), "0000152b-1212-efde-1523-785feabcd123" /* WEDO2_DISCONNECT */);
    }
    /**
     * Set the name of the Hub.
     * @param {string} name New name of the hub (14 characters or less, ASCII only).
     * @returns {Promise} Resolved upon successful issuance of command.
     */
    setName(name) {
      if (name.length > 14) {
        throw new Error("Name must be 14 characters or less");
      }
      return new Promise((resolve) => {
        const data = Buffer.from(name, "ascii");
        this.send(data, "00001524-1212-efde-1523-785feabcd123" /* WEDO2_NAME_ID */);
        this.send(data, "00001524-1212-efde-1523-785feabcd123" /* WEDO2_NAME_ID */);
        this._name = name;
        return resolve();
      });
    }
    send(message, uuid) {
      if (debug12.enabled) {
        debug12(`Sent Message (${this._getCharacteristicNameFromUUID(uuid)})`, message);
      }
      return this._bleDevice.writeToCharacteristic(uuid, message);
    }
    subscribe(portId, deviceType, mode) {
      this.send(Buffer.from([1, 2, portId, deviceType, mode, 1, 0, 0, 0, 0, 1]), "00001563-1212-efde-1523-785feabcd123" /* WEDO2_PORT_TYPE_WRITE */);
    }
    unsubscribe(portId, deviceType, mode) {
      this.send(Buffer.from([1, 2, portId, deviceType, mode, 1, 0, 0, 0, 0, 0]), "00001563-1212-efde-1523-785feabcd123" /* WEDO2_PORT_TYPE_WRITE */);
    }
    _getCharacteristicNameFromUUID(uuid) {
      const keys = Object.keys(BLECharacteristic);
      for (let i = 0; i < keys.length; i++) {
        const key = keys[i];
        if (BLECharacteristic[key] === uuid) {
          return key;
        }
      }
      return "UNKNOWN";
    }
    _parseHighCurrentAlert(data) {
      debug12("Received Message (WEDO2_HIGH_CURRENT_ALERT)", data);
    }
    _parseBatteryMessage(data) {
      debug12("Received Message (WEDO2_BATTERY)", data);
      const batteryLevel = data[0];
      if (batteryLevel !== this._batteryLevel) {
        this._batteryLevel = batteryLevel;
        this.emit("batteryLevel", { batteryLevel });
      }
    }
    _parseFirmwareRevisionString(data) {
      debug12("Received Message (WEDO2_FIRMWARE_REVISION)", data);
      this._firmwareVersion = data.toString();
    }
    _parsePortMessage(data) {
      debug12("Received Message (WEDO2_PORT_TYPE)", data);
      const portId = data[0];
      const event = data[1];
      const deviceType = event ? data[3] : 0;
      if (event === 1) {
        const device = this._createDevice(deviceType, portId);
        this._attachDevice(device);
      } else if (event === 0) {
        const device = this._getDeviceByPortId(portId);
        if (device) {
          this._detachDevice(device);
        }
      }
    }
    _parseSensorMessage(message) {
      debug12("Received Message (WEDO2_SENSOR_VALUE)", message);
      if (message[0] === 1) {
        this.emit("button", { event: 2 /* PRESSED */ });
        return;
      } else if (message[0] === 0) {
        this.emit("button", { event: 0 /* RELEASED */ });
        return;
      }
      const portId = message[1];
      const device = this._getDeviceByPortId(portId);
      if (device) {
        device.receive(message);
      }
    }
  };
  var PortMap7 = {
    "A": 1,
    "B": 2,
    "CURRENT_SENSOR": 3,
    "VOLTAGE_SENSOR": 4,
    "PIEZO_BUZZER": 5,
    "HUB_LED": 6
  };

  // src/poweredup-browser.ts
  var import_events4 = __toESM(require_events(), 1);
  var import_debug12 = __toESM(require_browser(), 1);

  // src/hubs/technicsmallhub.ts
  var import_debug11 = __toESM(require_browser(), 1);
  var debug13 = (0, import_debug11.default)("hub");
  var TechnicSmallHub = class extends LPF2Hub {
    constructor(device) {
      super(device, PortMap8, 8 /* TECHNIC_SMALL_HUB */);
      this._currentPort = 59;
      debug13("Discovered Spike Essential Hub");
    }
    static IsTechnicSmallHub(peripheral) {
      return peripheral.advertisement && peripheral.advertisement.serviceUuids && peripheral.advertisement.serviceUuids.indexOf("00001623-1212-efde-1623-785feabcd123" /* LPF2_HUB */.replace(/-/g, "")) >= 0 && peripheral.advertisement.manufacturerData && peripheral.advertisement.manufacturerData.length > 3 && peripheral.advertisement.manufacturerData[3] === 131 /* TECHNIC_SMALL_HUB_ID */;
    }
    async connect() {
      debug13("Connecting to Spike Essential Hub");
      await super.connect();
      debug13("Connect completed");
    }
  };
  var PortMap8 = {
    "A": 0,
    "B": 1,
    "HUB_LED": 49,
    "CURRENT_SENSOR": 59,
    "VOLTAGE_SENSOR": 60,
    "ACCELEROMETER": 97,
    "GYRO_SENSOR": 98,
    "TILT_SENSOR": 99
  };

  // src/poweredup-browser.ts
  var debug14 = (0, import_debug12.default)("poweredup");
  var PoweredUP = class extends import_events4.EventEmitter {
    constructor() {
      super();
      this._connectedHubs = {};
      this._discoveryEventHandler = this._discoveryEventHandler.bind(this);
    }
    /**
     * Begin scanning for Powered UP Hub devices.
     */
    async scan() {
      try {
        const device = await navigator.bluetooth.requestDevice({
          filters: [
            {
              services: [
                "00001523-1212-efde-1523-785feabcd123" /* WEDO2_SMART_HUB */
              ]
            },
            {
              services: [
                "00001623-1212-efde-1623-785feabcd123" /* LPF2_HUB */
              ]
            }
          ],
          optionalServices: [
            "00004f0e-1212-efde-1523-785feabcd123" /* WEDO2_SMART_HUB_2 */,
            "battery_service",
            "device_information"
          ]
        });
        const server = await device.gatt.connect();
        this._discoveryEventHandler.call(this, server);
        return true;
      } catch (err) {
        return false;
      }
    }
    /**
     * Retrieve a list of Powered UP Hubs.
     * @returns {BaseHub[]}
     */
    getHubs() {
      return Object.values(this._connectedHubs);
    }
    /**
     * Retrieve a Powered UP Hub by UUID.
     * @param {string} uuid
     * @returns {BaseHub | null}
     */
    getHubByUUID(uuid) {
      return this._connectedHubs[uuid];
    }
    /**
     * Retrieve a Powered UP Hub by primary MAC address.
     * @param {string} address
     * @returns {BaseHub}
     */
    getHubByPrimaryMACAddress(address) {
      return Object.values(this._connectedHubs).filter((hub) => hub.primaryMACAddress === address)[0];
    }
    /**
     * Retrieve a list of Powered UP Hub by name.
     * @param {string} name
     * @returns {BaseHub[]}
     */
    getHubsByName(name) {
      return Object.values(this._connectedHubs).filter((hub) => hub.name === name);
    }
    /**
     * Retrieve a list of Powered UP Hub by type.
     * @param {string} name
     * @returns {BaseHub[]}
     */
    getHubsByType(hubType) {
      return Object.values(this._connectedHubs).filter((hub) => hub.type === hubType);
    }
    _determineLPF2HubType(device) {
      return new Promise(async (resolve) => {
        let buf = Buffer.alloc(0);
        await device.subscribeToCharacteristic("00001624-1212-efde-1623-785feabcd123" /* LPF2_ALL */, (data) => {
          buf = Buffer.concat([buf, data]);
          while (buf[0] <= buf.length) {
            const len = buf[0];
            const message = buf.slice(0, len);
            buf = buf.slice(len);
            if (message[2] === 1 && message[3] === 11) {
              switch (message[5]) {
                case 66 /* REMOTE_CONTROL_ID */:
                  resolve(4 /* REMOTE_CONTROL */);
                  break;
                case 64 /* MOVE_HUB_ID */:
                  resolve(2 /* MOVE_HUB */);
                  break;
                case 65 /* HUB_ID */:
                  resolve(3 /* HUB */);
                  break;
                case 32 /* DUPLO_TRAIN_BASE_ID */:
                  resolve(5 /* DUPLO_TRAIN_BASE */);
                  break;
                case 131 /* TECHNIC_SMALL_HUB_ID */:
                  resolve(8 /* TECHNIC_SMALL_HUB */);
                  break;
                case 128 /* TECHNIC_MEDIUM_HUB_ID */:
                  resolve(6 /* TECHNIC_MEDIUM_HUB */);
                  break;
                case 67 /* MARIO_ID */:
                  resolve(7 /* MARIO */);
                  break;
              }
              debug14("Hub type determined");
            } else {
              debug14("Stashed in mailbox (LPF2_ALL)", message);
              device.addToCharacteristicMailbox("00001624-1212-efde-1623-785feabcd123" /* LPF2_ALL */, message);
            }
          }
        });
        device.writeToCharacteristic("00001624-1212-efde-1623-785feabcd123" /* LPF2_ALL */, Buffer.from([5, 0, 1, 11, 5]));
      });
    }
    async _discoveryEventHandler(server) {
      const device = new WebBLEDevice(server);
      let hub;
      let hubType = 0 /* UNKNOWN */;
      let isLPF2Hub = false;
      try {
        await device.discoverCharacteristicsForService("00001523-1212-efde-1523-785feabcd123" /* WEDO2_SMART_HUB */);
        hubType = 1 /* WEDO2_SMART_HUB */;
      } catch (error) {
      }
      try {
        if (hubType !== 1 /* WEDO2_SMART_HUB */) {
          await device.discoverCharacteristicsForService("00001623-1212-efde-1623-785feabcd123" /* LPF2_HUB */);
          isLPF2Hub = true;
        }
      } catch (error) {
      }
      if (isLPF2Hub) {
        hubType = await this._determineLPF2HubType(device);
      }
      switch (hubType) {
        case 1 /* WEDO2_SMART_HUB */:
          hub = new WeDo2SmartHub(device);
          break;
        case 2 /* MOVE_HUB */:
          hub = new MoveHub(device);
          break;
        case 3 /* HUB */:
          hub = new Hub(device);
          break;
        case 4 /* REMOTE_CONTROL */:
          hub = new RemoteControl(device);
          break;
        case 5 /* DUPLO_TRAIN_BASE */:
          hub = new DuploTrainBase(device);
          break;
        case 8 /* TECHNIC_SMALL_HUB */:
          hub = new TechnicSmallHub(device);
          break;
        case 6 /* TECHNIC_MEDIUM_HUB */:
          hub = new TechnicMediumHub(device);
          break;
        case 7 /* MARIO */:
          hub = new Mario(device);
          break;
        default:
          return;
      }
      device.on("discoverComplete", () => {
        hub.on("connect", () => {
          debug14(`Hub ${hub.uuid} connected`);
          this._connectedHubs[hub.uuid] = hub;
        });
        hub.on("disconnect", () => {
          debug14(`Hub ${hub.uuid} disconnected`);
          delete this._connectedHubs[hub.uuid];
        });
        debug14(`Hub ${hub.uuid} discovered`);
        this.emit("discover", hub);
      });
    }
  };

  // src/index-browser.ts
  var globalRef = globalThis;
  if (!globalRef.Buffer) {
    globalRef.Buffer = import_buffer.Buffer;
  }
  window.PoweredUP = {
    PoweredUP,
    BaseHub,
    WeDo2SmartHub,
    TechnicMediumHub,
    Hub,
    RemoteControl,
    DuploTrainBase,
    Consts: consts_exports,
    Color: Color2,
    ColorDistanceSensor,
    Device,
    DuploTrainBaseColorSensor,
    DuploTrainBaseMotor,
    DuploTrainBaseSpeaker,
    DuploTrainBaseSpeedometer,
    HubLED,
    Light,
    Mario,
    MediumLinearMotor,
    MotionSensor,
    MoveHub,
    MoveHubMediumLinearMotor,
    MoveHubTiltSensor,
    PiezoBuzzer,
    RemoteControlButton,
    SimpleMediumLinearMotor,
    TechnicColorSensor,
    TechnicDistanceSensor,
    TechnicForceSensor,
    TechnicMediumHubAccelerometerSensor,
    TechnicMediumHubGyroSensor,
    TechnicMediumHubTiltSensor,
    TechnicSmallAngularMotor,
    TechnicMediumAngularMotor,
    TechnicLargeAngularMotor,
    TechnicLargeLinearMotor,
    TechnicXLargeLinearMotor,
    Technic3x3ColorLightMatrix,
    TiltSensor,
    TrainMotor,
    VoltageSensor,
    CurrentSensor,
    TachoMotor,
    AbsoluteMotor,
    BasicMotor,
    isWebBluetooth
  };
})();
/*! Bundled license information:

ieee754/index.js:
  (*! ieee754. BSD-3-Clause License. Feross Aboukhadijeh <https://feross.org/opensource> *)

buffer/index.js:
  (*!
   * The buffer module from node.js, for the browser.
   *
   * @author   Feross Aboukhadijeh <https://feross.org>
   * @license  MIT
   *)
*/
//# sourceMappingURL=poweredup.js.map
