# Simulated phone screens

`target:=rgb` keeps `phone_screen.obj` and its RGB screenshot. `target:=captcha`
selects `phone_captcha.obj`: the same plane and UVs with a screenshot of the
original English Google reCAPTCHA v2 test widget in `robot-touch-test-app`.
It contains one unchecked checkbox. Body, screen frames and collision boxes
are shared; only the visible texture changes. The phone must be enabled with
`right_virtual_phone:=true`.

The CAPTCHA texture is a static Android-emulator screenshot (1080 × 2400).
It cannot change the widget state or create Android events when touched in
Gazebo. Its warning identifies the public Google test key. The vision crop
in `denso_vision/assets/captcha_checkbox.png` comes from this screenshot at
`x=171..570`, `y=1186..1265`; its checkbox center is `(40.5, 39.5)` in crop
pixels and its side is approximately 73.5 pixels. Update both template and
offsets if the widget layout changes.
