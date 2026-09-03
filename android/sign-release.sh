#!/usr/bin/env bash
# Reframe360 Editor -- 360 video stabiliser and stitcher for dual-fisheye footage.
# Copyright (C) 2026 Peter Allen
# SPDX-License-Identifier: GPL-3.0-or-later
#
# This program is free software under the GNU General Public License, version
# 3 or (at your option) any later version; see LICENSE for the full text.
# It is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY, without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.

# Sign an already-built .aab or .apk with the release key.
#
#   android/sign-release.sh reframe360-release-unsigned.aab
#   android/sign-release.sh some.apk out.apk
#
# release-android.sh signs as it builds; this is for an artifact that came out
# unsigned -- a CI build, or a local build made without the keystore to hand.
#
# THE DEBUG SIGNATURE MUST GO FIRST. androiddeployqt signs its output with the
# Android debug key (you can see it as META-INF/ANDROIDD.{SF,RSA}), so a
# "release-unsigned" file is not unsigned at all. jarsigner would happily ADD a
# second signature and leave the debug one in place, and Play rejects a bundle
# that carries a signer it is not expecting. So the old signature files are
# stripped before the real one goes on.
#
# Bundles are signed with jarsigner -- apksigner only handles APKs -- and APKs
# with apksigner, which writes the v2/v3 block a modern Android needs and which
# jarsigner cannot produce.
#
# Keep ~/.android/render360-release.keystore safe: Android only accepts UPDATES
# signed by the same key, so losing it means every user must uninstall (losing
# their folder grant and settings) before they can move to a new build.
set -euo pipefail

IN=${1:-}
if [ -z "$IN" ] || [ ! -f "$IN" ]; then
    echo "usage: $0 <file.aab|file.apk> [output]" >&2
    exit 2
fi

export JAVA_HOME=${JAVA_HOME:-/usr/lib/jvm/java-21-openjdk-amd64}
export PATH=$JAVA_HOME/bin:$PATH
export ANDROID_SDK_ROOT=${ANDROID_SDK_ROOT:-$HOME/Android/Sdk}

KEYSTORE=${KEYSTORE:-$HOME/.android/render360-release.keystore}
KEYSTORE_PASS=${KEYSTORE_PASS:-render360}
KEY_ALIAS=${KEY_ALIAS:-render360}
APKSIGNER=${APKSIGNER:-$(ls -d "$ANDROID_SDK_ROOT"/build-tools/*/apksigner 2>/dev/null | sort -V | tail -1 || true)}
ZIPALIGN=${ZIPALIGN:-$(ls -d "$ANDROID_SDK_ROOT"/build-tools/*/zipalign 2>/dev/null | sort -V | tail -1 || true)}

[ -f "$KEYSTORE" ] || { echo "no keystore at $KEYSTORE" >&2; exit 1; }
[ -x "$JAVA_HOME/bin/jarsigner" ] || { echo "no jarsigner at $JAVA_HOME/bin" >&2; exit 1; }

case "$IN" in
    *.aab) KIND=aab ;;
    *.apk) KIND=apk ;;
    *) echo "not an .aab or .apk: $IN" >&2; exit 2 ;;
esac

# Default output drops any "-unsigned" from the name rather than adding to it.
DEFAULT_OUT=$(dirname "$IN")/$(basename "$IN" ".$KIND" | sed 's/-unsigned$//')-signed.$KIND
OUT=${2:-$DEFAULT_OUT}

# Signing in place is legitimate (release-android.sh does exactly that), so
# only copy when there is somewhere to copy to.
if [ "$(readlink -f "$IN")" != "$(readlink -f "$OUT")" ]; then
    cp -f "$IN" "$OUT"
fi

# Strip whatever signed it before (the debug key, normally). `zip -d` returns
# 12 when nothing matched, which is fine -- a genuinely unsigned file.
OLD=$(unzip -l "$OUT" | awk '{print $4}' | grep -E '^META-INF/.*\.(SF|RSA|DSA|EC)$' || true)
if [ -n "$OLD" ]; then
    echo "stripping existing signature: $(echo "$OLD" | tr '\n' ' ')"
    zip -q -d "$OUT" 'META-INF/*.SF' 'META-INF/*.RSA' 'META-INF/*.DSA' 'META-INF/*.EC' \
                     'META-INF/MANIFEST.MF' || [ $? -eq 12 ]
fi

if [ "$KIND" = aab ]; then
    "$JAVA_HOME/bin/jarsigner" -keystore "$KEYSTORE" \
        -storepass "$KEYSTORE_PASS" -keypass "$KEYSTORE_PASS" \
        -sigalg SHA256withRSA -digestalg SHA-256 \
        "$OUT" "$KEY_ALIAS" > /dev/null
    # Verified WITHOUT -strict. An Android signing key is self-signed and
    # chains to no CA, which -strict reports as a signer error (exit 4) --
    # true, and entirely normal for this purpose.
    "$JAVA_HOME/bin/jarsigner" -verify "$OUT" | grep -q "jar verified" \
        || { echo "jarsigner could not verify $OUT" >&2; exit 1; }

    # It has to be signed by OUR key and nothing else. A leftover debug
    # signature is the failure this script exists to prevent, so check rather
    # than assume the strip worked.
    LEFT=$(unzip -l "$OUT" | awk '{print $4}' | grep -E '^META-INF/.*\.(SF|RSA|DSA|EC)$' || true)
    NSIG=$(echo "$LEFT" | grep -c '\.\(RSA\|DSA\|EC\)$' || true)
    if [ "$NSIG" != "1" ]; then
        echo "expected exactly one signature, found: $(echo "$LEFT" | tr '\n' ' ')" >&2
        exit 1
    fi

    echo
    echo "signed: $OUT ($(du -h "$OUT" | cut -f1))"
    echo "  signature: $(echo "$LEFT" | tr '\n' ' ')"
    # The fingerprint, so it can be checked against the upload key Play expects
    # BEFORE uploading rather than after a rejection.
    keytool -list -keystore "$KEYSTORE" -storepass "$KEYSTORE_PASS" -alias "$KEY_ALIAS" -v 2>/dev/null \
        | grep -E "^Owner:|SHA256:" | sed 's/^/  /'
else
    [ -n "$APKSIGNER" ] || { echo "no apksigner in $ANDROID_SDK_ROOT/build-tools" >&2; exit 1; }
    if [ -n "$ZIPALIGN" ]; then
        "$ZIPALIGN" -p -f 4 "$OUT" "$OUT.aligned" && mv -f "$OUT.aligned" "$OUT"
    fi
    "$APKSIGNER" sign --ks "$KEYSTORE" --ks-pass "pass:$KEYSTORE_PASS" \
        --key-pass "pass:$KEYSTORE_PASS" --ks-key-alias "$KEY_ALIAS" "$OUT"
    "$APKSIGNER" verify --print-certs "$OUT" | sed 's/^/  /'
    echo "signed: $OUT ($(du -h "$OUT" | cut -f1))"
fi
