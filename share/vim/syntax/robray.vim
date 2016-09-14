"  VIM Syntax File for Amino Scene Files
"
"     robray.vim -- defines syntax highlighting from seml files
"     Copyright (c) 2016, Rice University
"
"   Redistribution and use in source and binary forms, with or
"   without modification, are permitted provided that the following
"   conditions are met:
"   * Redistributions of source code must retain the above copyright
"     notice, this list of conditions and the following disclaimer.
"   * Redistributions in binary form must reproduce the above
"     copyright notice, this list of conditions and the following
"     disclaimer in the documentation and/or other materials provided
"     with the distribution.
"   * Neither the name of copyright holder the names of its
"     contributors may be used to endorse or promote products derived
"     from this software without specific prior written permission.
"
"   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
"   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
"   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
"   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
"   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
"   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
"   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
"   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
"   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
"   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
"   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
"   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
"   POSSIBILITY OF SUCH DAMAGE.

syn keyword robrayFrameAttribute parent translation quaternion rpy type axis offset variable
syn keyword robrayGeomAttribute shape mesh color alpha specular dimension radius height start_radius end_radius delta thickness isa scale visual collision
syn keyword robrayKeyword frame class geometry allow_collision
syn keyword robrayInclude include
syn keyword robrayDef     def
syn keyword robrayShape grid box cone cylinder sphere
syn keyword robrayJoint fixed prismatic revolute
syn keyword robrayPi pi

syn match robrayLineComment "\(//\|#\).*$"
syn region robrayBlockComment start="/\*" end="\*/"

syn region robrayString start='"' end='"'

syn match       robrayFloat          "\d\+"
"Float Matching adapted from c.vim
"floating point number, with dot, optional exponent
syn match       robrayFloat          "\d\+\.\d*\([eds][-+]\=\d\+\)\=[fl]\="
"floating point number, starting with a dot, optional exponent
syn match       robrayFloat          "\.\d\+\([eds][-+]\=\d\+\)\=[fl]\=\>"
"floating point number, without dot, with exponent
syn match       robrayFloat          "\d\+[eds][-+]\=\d\+[fl]\=\>"


syn match       robrayId          "[a-zA-Z][a-zA-Z0-9_\-]\+"

hi def link robrayLineComment   comment
hi def link robrayBlockComment   comment

hi def link robrayKeyword     keyword
hi def link robrayInclude     include
hi def link robrayDef         define
hi def link robrayFrameAttribute   type
hi def link robrayGeomAttribute   type

hi def link robrayShape       constant
hi def link robrayJoint       constant
hi def link robrayPi          float
hi def link robrayString      string
hi def link robrayFloat       float

hi def link robrayId          identifier
