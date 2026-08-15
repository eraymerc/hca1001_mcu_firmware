package main

import (
	"image/color"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/theme"
)

// Palette carried over from the Python GUI's dark_theme.DARK_QSS so the two
// front ends read as the same instrument.
var (
	colBackground = color.NRGBA{R: 0x1e, G: 0x1e, B: 0x1e, A: 0xff}
	colSurface    = color.NRGBA{R: 0x26, G: 0x26, B: 0x26, A: 0xff}
	colButton     = color.NRGBA{R: 0x2d, G: 0x2d, B: 0x2d, A: 0xff}
	colInput      = color.NRGBA{R: 0x2a, G: 0x2a, B: 0x2a, A: 0xff}
	colHover      = color.NRGBA{R: 0x3a, G: 0x3a, B: 0x3a, A: 0xff}
	colPressed    = color.NRGBA{R: 0x45, G: 0x45, B: 0x45, A: 0xff}
	colBorder     = color.NRGBA{R: 0x45, G: 0x45, B: 0x45, A: 0xff}
	colSeparator  = color.NRGBA{R: 0x3a, G: 0x3a, B: 0x3a, A: 0xff}
	colText       = color.NRGBA{R: 0xe0, G: 0xe0, B: 0xe0, A: 0xff}
	colDisabled   = color.NRGBA{R: 0x6a, G: 0x6a, B: 0x6a, A: 0xff}
	colAccent     = color.NRGBA{R: 0x00, G: 0xd0, B: 0xff, A: 0xff}
	colHeading    = color.NRGBA{R: 0x9f, G: 0xd3, B: 0xff, A: 0xff}
	colOK         = color.NRGBA{R: 0x6f, G: 0xe0, B: 0x8a, A: 0xff}
	colError      = color.NRGBA{R: 0xff, G: 0x80, B: 0x80, A: 0xff}
	colWarning    = color.NRGBA{R: 0xff, G: 0xd4, B: 0x00, A: 0xff}
)

type darkTheme struct{}

var _ fyne.Theme = darkTheme{}

func (darkTheme) Color(name fyne.ThemeColorName, _ fyne.ThemeVariant) color.Color {
	switch name {
	case theme.ColorNameBackground:
		return colBackground
	case theme.ColorNameButton:
		return colButton
	case theme.ColorNameDisabledButton:
		return colSurface
	case theme.ColorNameDisabled:
		return colDisabled
	case theme.ColorNameError:
		return colError
	case theme.ColorNameFocus:
		return color.NRGBA{R: 0x00, G: 0xd0, B: 0xff, A: 0x66}
	case theme.ColorNameForeground:
		return colText
	case theme.ColorNameForegroundOnPrimary:
		return colBackground
	case theme.ColorNameHeaderBackground:
		return colSurface
	case theme.ColorNameHover:
		return colHover
	case theme.ColorNameHyperlink:
		return colAccent
	case theme.ColorNameInputBackground:
		return colInput
	case theme.ColorNameInputBorder:
		return colBorder
	case theme.ColorNameMenuBackground:
		return colSurface
	case theme.ColorNameOverlayBackground:
		return colSurface
	case theme.ColorNamePlaceHolder:
		return colDisabled
	case theme.ColorNamePressed:
		return colPressed
	case theme.ColorNamePrimary:
		return colAccent
	case theme.ColorNameScrollBar:
		return colBorder
	case theme.ColorNameSelection:
		return color.NRGBA{R: 0x3a, G: 0x5a, B: 0x7a, A: 0xff}
	case theme.ColorNameSeparator:
		return colSeparator
	case theme.ColorNameShadow:
		return color.NRGBA{A: 0x66}
	case theme.ColorNameSuccess:
		return colOK
	case theme.ColorNameWarning:
		return colWarning
	}
	return theme.DefaultTheme().Color(name, theme.VariantDark)
}

func (darkTheme) Font(style fyne.TextStyle) fyne.Resource {
	return theme.DefaultTheme().Font(style)
}

func (darkTheme) Icon(name fyne.ThemeIconName) fyne.Resource {
	return theme.DefaultTheme().Icon(name)
}

func (darkTheme) Size(name fyne.ThemeSizeName) float32 {
	switch name {
	case theme.SizeNamePadding:
		return 4
	case theme.SizeNameInnerPadding:
		return 6
	case theme.SizeNameText:
		return 13
	case theme.SizeNameInputBorder:
		return 1
	}
	return theme.DefaultTheme().Size(name)
}
