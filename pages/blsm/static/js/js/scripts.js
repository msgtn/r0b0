/*!
* Start Bootstrap - Freelancer v7.0.7 (https://startbootstrap.com/theme/freelancer)
* Copyright 2013-2023 Start Bootstrap
* Licensed under MIT (https://github.com/StartBootstrap/startbootstrap-freelancer/blob/master/LICENSE)
*/
//
// Scripts
// 

window.addEventListener('DOMContentLoaded', event => {

    // Navbar shrink function
    var navbarShrink = function () {
        const navbarCollapsible = document.body.querySelector('#mainNav');
        if (!navbarCollapsible) {
            return;
        }
        if (window.scrollY === 0) {
            navbarCollapsible.classList.remove('navbar-shrink')
        } else {
            navbarCollapsible.classList.add('navbar-shrink')
        }

    };

    // Shrink the navbar 
    navbarShrink();

    // Shrink the navbar when page is scrolled
    document.addEventListener('scroll', navbarShrink);

    // Activate Bootstrap scrollspy on the main nav element
    const mainNav = document.body.querySelector('#mainNav');
    if (mainNav) {
        new bootstrap.ScrollSpy(document.body, {
            target: '#mainNav',
            rootMargin: '0px 0px -40%',
        });
    };

    // Collapse responsive navbar when toggler is visible
    const navbarToggler = document.body.querySelector('.navbar-toggler');
    const responsiveNavItems = [].slice.call(
        document.querySelectorAll('#navbarResponsive .nav-link')
    );
    responsiveNavItems.map(function (responsiveNavItem) {
        responsiveNavItem.addEventListener('click', () => {
            if (window.getComputedStyle(navbarToggler).display !== 'none') {
                navbarToggler.click();
            }
        });
    });

});

// state indicator in navbar
(function () {
    const ENDPOINT = '/api/state';
    let desiredState = null;
    let stateSocket = null;

    function applyLabelStyle(label, status, hover) {
        // Ensure rectangle outline and hover emphasis
        const styles = {
            default: { // The border is now on the 'a' tag, so we only manage background and text color here.
                border: '#6c757d',
                text: '',
                bg: 'rgba(108,117,125,0.10)',
                bgHover: 'rgba(108,117,125,0.18)'
            }, // Note: border color is now managed on the 'a' tag directly.
            ok: {
                border: '#12dc41ff',
                text: '#12dc41ff',
                bg: 'rgba(40,167,69,0.10)',
                bgHover: 'rgba(40,167,69,0.20)'
            },
            warn: {
                border: '#ffc107',
                text: '#ffc107',
                bg: 'rgba(255,193,7,0.12)',
                bgHover: 'rgba(255,193,7,0.22)'
            },
            error: {
                border: '#dc3545',
                text: '#dc3545',
                bg: 'rgba(220,53,69,0.10)',
                bgHover: 'rgba(220,53,69,0.20)'
            }
        };
        const key = status === 'ok' || status === 'warn' || status === 'error' ? status : 'default';
        const cfg = styles[key];
        label.style.borderColor = cfg.border;
        label.style.color = cfg.text;
        label.style.backgroundColor = hover ? cfg.bgHover : cfg.bg;
    }

    function createNavItem(navList) {
        if (!navList || document.getElementById('r0b0-state-nav-item')) return null;

        const li = document.createElement('li');
        li.className = 'nav-item mx-0 mx-lg-1';
        li.id = 'r0b0-state-nav-item';

    const a = document.createElement('a');
    // Use flex (not inline-flex) to inherit full nav-link height; center contents vertically
    a.className = 'nav-link py-3 px-0 px-lg-3 rounded d-flex align-items-center';
    a.href = '#';
    a.setAttribute('aria-current', 'page');
    a.id = 'r0b0-state-nav-link';
    // Prevent uppercase transform inherited from parent navbar
    a.style.textTransform = 'none';
        // Dot indicator
        const dot = document.createElement('span');
        dot.setAttribute('aria-hidden', 'true');
        dot.style.display = 'inline-block';
        dot.style.width = '0.5rem';
        dot.style.height = '0.5rem';
        dot.style.borderRadius = '50%';
        dot.style.backgroundColor = '#6c757d'; // default muted
        dot.style.marginRight = '0.5rem';
        dot.style.verticalAlign = 'middle';

    // Label (rectangle outline with subtle background)
        const label = document.createElement('span');
    label.textContent = 'state: unknown';
    label.style.verticalAlign = 'middle';
    label.style.fontWeight = '600';
    label.style.display = 'inline-flex';
    label.style.alignItems = 'center';
    label.style.marginLeft = '0.25rem';
    label.style.fontSize = '1.0rem'; // a bit larger as requested
    label.style.lineHeight = '1'; // keep tight so nav-link controls vertical alignment
    label.style.textTransform = 'none';
    // Keep vertical padding at 0 to maintain centerline; increase horizontal padding for a bigger box
    label.style.padding = '0 1.0rem';
    label.style.border = '2px solid'; // color will be set by applyLabelStyle
    label.style.borderRadius = '0.35rem'; // rectangle (slightly rounded corners)
    label.style.backgroundColor = 'rgba(108,117,125,0.10)';

        a.appendChild(dot);
        a.appendChild(label);
        li.appendChild(a);

    // Hover effect color change based on current status
    a.addEventListener('mouseenter', () => applyLabelStyle(label, li.dataset.status || 'default', true));
    a.addEventListener('mouseleave', () => applyLabelStyle(label, li.dataset.status || 'default', false));

        if (navList.firstChild) {
            navList.insertBefore(li, navList.firstChild);
        } else {
            navList.appendChild(li);
        }
        return { li, a, dot, label };
    }

    function setVisualState(parts, stateText, status) {
        if (!parts) return;
        const { dot, label, li } = parts;
    label.textContent = `state: ${stateText}`;
        switch (status) {
            case 'ok':
                dot.style.background = '#28a745'; // green
                applyLabelStyle(label, 'ok', false);
                break;
            case 'warn':
                dot.style.background = '#ffc107'; // amber
                applyLabelStyle(label, 'warn', false);
                break;
            case 'error':
                dot.style.background = '#dc3545'; // red
                applyLabelStyle(label, 'error', false);
                break;
            default:
                dot.style.background = '#6c757d'; // gray
                applyLabelStyle(label, 'default', false);
        }
        // Persist status on element for hover handling
        li.dataset.status = (status === 'ok' || status === 'warn' || status === 'error') ? status : 'default';
    }

    // Determine desired state based on current page path
    //----- UPDATE THIS FOR NEW STATE----------
    // here we list all states that we have, need to keep in sync with page.py
    function detectDesiredState() {
        const path = window.location.pathname || '';
        if (path.startsWith('/blsm_web')) return 'key_control';
        if (path.startsWith('/blsm_sensor')) return 'sensor';
        if (path.startsWith('/blsm_playback')) return 'playback';
        if (path.startsWith('/blsm_calib')) return 'calibration';
        if (path.startsWith('/speech_recognition')) return 'speech';
        return 'idle';
    }


    function start() {
        const navList = document.querySelector('#navbarResponsive .navbar-nav');
        if (!navList) return;
        const parts = createNavItem(navList);
        if (!parts) return;
        // Determine and optimistically show desired state for this page
        desiredState = detectDesiredState();
        setVisualState(parts, desiredState, 'ok');

        // Socket.IO: set state via socket and receive authoritative updates
        try {
            if (typeof io === 'function') {
                stateSocket = io();

                // Apply authoritative updates from server
                stateSocket.on('state_update', ({ state }) => {
                    setVisualState(parts, (typeof state === 'string' ? state : 'unknown'), 'ok');
                });

                // When connected, request state change via socket with ack
                const sendDesired = () => {
                    if (!desiredState) return;
                    try {
                        stateSocket.emit('set_state', { state: desiredState }, (resp) => {
                            if (resp && resp.error) {
                                // Show warning if server rejected the state
                                setVisualState(parts, desiredState, 'warn');
                            } else if (resp && typeof resp.state === 'string') {
                                setVisualState(parts, resp.state, 'ok');
                            }
                        });
                    } catch (_) {
                        // ignore
                    }
                };

                stateSocket.on('connect', sendDesired);
                // If already connected, send immediately
                if (stateSocket.connected) sendDesired();
            }
        } catch (e) {
            // ignore if socket.io not available on this page
        }
    }

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', start);
    } else {
        start();
    }
})();
