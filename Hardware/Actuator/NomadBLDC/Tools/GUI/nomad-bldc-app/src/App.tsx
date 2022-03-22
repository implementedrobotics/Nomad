// React
import React from 'react';

// Router
import { Route, BrowserRouter as Router, Routes, useNavigate } from 'react-router-dom';

// Pages
import Home from './Pages/Home';
import RealtimeVisualizer from './Pages/RealtimeVisualizer';
import Settings from './Pages/Settings';

// MUI
import { Box, Paper, ListItemButton, ListItemIcon, ListItemText } from '@mui/material';

// Icons
import DashboardIcon from '@mui/icons-material/Dashboard';
import TimelineIcon from '@mui/icons-material/Timeline';
import SettingsIcon from '@mui/icons-material/Settings';

import ErrorPage from 'Pages/ErrorPage';

function NavigationMenu() {
    const navigate = useNavigate();

    const [selectedIndex, setSelectedIndex] = React.useState(0);

    return (
        <>
            <ListItemButton
                onClick={() => {
                    navigate('/');
                    setSelectedIndex(0);
                }}
                selected={selectedIndex === 0}
                disableRipple
            >
                <ListItemIcon>
                    <DashboardIcon />
                </ListItemIcon>
                <ListItemText primary="Device Dashboard" />
            </ListItemButton>
            <ListItemButton
                onClick={() => {
                    navigate('realtime');
                    setSelectedIndex(1);
                }}
                selected={selectedIndex === 1}
                disableRipple
            >
                <ListItemIcon>
                    <TimelineIcon />
                </ListItemIcon>
                <ListItemText primary="Realtime Telemetry" />
            </ListItemButton>
            <ListItemButton
                onClick={() => {
                    navigate('settings');
                    setSelectedIndex(2);
                }}
                selected={selectedIndex === 2}
                disableRipple
            >
                <ListItemIcon>
                    <SettingsIcon />
                </ListItemIcon>
                <ListItemText primary="Settings" />
            </ListItemButton>
        </>
    );
}
function App() {
    return (
        <Router>
            <Box sx={{ display: 'flex', backgroundColor: '#EEEEEE', height: '100vh' }}>
                <Paper
                    elevation={4}
                    square={false}
                    sx={{
                        width: 240,
                        margin: '12px',
                    }}
                >
                    <NavigationMenu />
                </Paper>
                <Paper
                    elevation={4}
                    square={false}
                    sx={{
                        margin: '12px',
                    }}
                >
                    <Routes>
                        <Route path="/" element={<Home />} />
                        <Route path="/realtime" element={<RealtimeVisualizer />} />
                        <Route path="/settings" element={<Settings />} />
                        <Route path="*" element={<ErrorPage />} />
                    </Routes>
                </Paper>
            </Box>
        </Router>
    );
}
export default App;
